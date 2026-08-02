# Target Detector Node — Architecture

`uav_vision/target_detector_node.py` runs YOLO target detection and ArUco
marker detection (marker mounted on the ground robot) on the UAV's D436
color+depth stream, and publishes the detected target's position relative
to the marker. It does **not** convert this into the ground robot's map
frame itself — that's `ground_controller/coordinate_bridge_node.py`'s job
(implemented; see "Downstream: coordinate_bridge_node" below), which
consumes `/camera/target_pose_raw` and republishes it in the `map` frame
on `/camera/target_pose` for the scheduler.

---

## I/O

**Subscribes:**
- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/Image`) —
  must be the *aligned* depth topic (`align_depth.enable:=true` on the
  realsense2_camera launch); the plain `depth/image_rect_raw` topic is not
  pixel-aligned to the color frame and will produce wrong depth readings.
- `/camera/camera/color/camera_info` (`sensor_msgs/CameraInfo`)

**Publishes:**
- `/camera/target_pose_raw` (`geometry_msgs/PoseStamped`, `frame_id` =
  the ArUco marker's frame, e.g. `aruco_marker`) — raw target position
  relative to the marker, **not** the map frame the scheduler expects on
  `/camera/target_pose` (see `coordinate_bridge_node.py` below for that
  conversion).
- `/camera/annotated_image/compressed` (`sensor_msgs/CompressedImage`,
  JPEG) — the color frame with the detected marker axes + target bounding
  box drawn on it, published alongside every successful detection. Feeds
  the web dashboard's live camera panel. JPEG-compressed rather than raw:
  rosbridge falls behind and silently drops most frames of a raw
  1280x720 image at even 1-2Hz (each frame is several MB once
  base64-encoded into JSON); a JPEG frame is a few dozen to a few hundred
  KB, which rosbridge keeps up with easily.

---

## Per-frame pipeline

```
New color frame arrives
    │
    ├─ no camera_info yet / no cached depth frame → skip, wait for next
    ├─ single_shot=true and already succeeded once → skip permanently
    │
    ▼
① Update ArUco marker observation
    - Run ArUco detection on the full frame, look for marker_id (default 0)
    - If found: solvePnP → marker pose relative to camera (rvec, tvec),
      timestamp this observation
    │
    ├─ marker never seen → skip, wait for next frame
    ├─ last marker sighting older than marker_max_age_sec (default 2.0s)
    │     → skip (stale reference, e.g. ground robot may have moved)
    ▼
② Tiled YOLO detection
    - Slice the frame into overlapping tiles (tile_size=256, tile_overlap=0.4)
      and run YOLO on each tile independently, not once on the full frame
      → the target is often a small fraction of the full frame and gets lost
        once YOLO downsamples the whole image to its input resolution
    - Collect all boxes where class == target_class_id (default 0 = "person")
    - Aspect-ratio filter: box width/height must fall within
      [min_aspect_ratio, max_aspect_ratio] (default [1.0, 5.0]) — rejects
      known taller-than-wide distractors (e.g. a small drone model in the
      test scene) that the real, wider-than-tall target never produces
    - Keep the highest-confidence surviving candidate
    │
    ├─ no candidate, or best confidence < detection_confidence_threshold
    │     (default 0.4) → this frame fails, skip, wait for next frame
    ▼
③ Backproject to the marker frame
    - Read the depth at the box's center pixel
    - Reject if outside [min_depth, max_depth] (default 0.1–2.5 m)
    - Backproject (pixel, depth) → 3D point in the camera frame using the
      live camera intrinsics (pinhole model)
    - Transform that point into the marker frame using the marker's pose
      from step ① (rigid transform, not a "camera is level" approximation —
      this is correct for any camera tilt as long as the marker pose and
      depth reading are accurate)
    ▼
④ Publish
    - Publish PoseStamped (relative to the marker) on /camera/target_pose_raw
    - Build the annotated frame (marker axes + target box drawn on the color
      image) and publish it as JPEG on /camera/annotated_image/compressed,
      regardless of save_tile_debug_dir (that's only for saving frames to
      disk for offline debugging - this publish always happens)
    - If save_tile_debug_dir is set: also write the annotated frame to disk
    - If single_shot=true: mark done, ignore all future frames
    - If single_shot=false: keep processing every incoming frame
```

---

## Design decisions and why

These came out of live testing against a real D436 + printed ArUco marker +
printed target photo on a benchtop maze mockup.

| Aspect | Decision | Reason |
|---|---|---|
| Tiled inference | 256px tiles, 40% overlap, per-tile YOLO call instead of one full-frame call | Full-frame inference downsamples the whole image to YOLO's input size; a small target loses all its detail and goes undetected |
| Camera intrinsics | Read live from `CameraInfo`, never hardcoded | Portable across cameras/resolutions |
| Marker staleness | `marker_max_age_sec` expires old marker sightings | Prevents computing a target position against a marker pose that's no longer valid (e.g. ground robot moved) |
| Confidence threshold | 0.4, not a lower value like 0.2 | At 0.2 a known false-positive object (a small drone model in the test scene) scores in the same range as genuine target detections — can't be separated by threshold alone |
| Aspect-ratio filter | box must be wider than tall (ratio ≥ 1.0) | The false-positive drone-model box is consistently taller-than-wide; the real (photo) target is consistently wider-than-tall. This shape difference is a more reliable discriminator than confidence alone |
| Retry across frames | A frame that fails any check is simply skipped; the node keeps trying on subsequent frames | Per-frame confidence has natural noise; waiting a few frames reliably crosses the threshold without any extra logic — this is inherent in the "keep listening until success" design, not a separate retry loop |
| `single_shot` | Stop entirely after the first successful detection | For a fixed (non-flying) camera, one confident detection is enough; continuously re-running the expensive tiled pass on every frame wastes compute |
| Backprojection math | Full 3D: real measured depth + real solvePnP pose, not a "camera is level" shortcut | Works correctly under any camera tilt — the marker pose and per-pixel depth are both measured, not assumed |

---

## Downstream: coordinate_bridge_node

`ground_controller/coordinate_bridge_node.py` (runs on the base station)
closes the gap this node deliberately leaves open:

1. Publishes a static `base_link → aruco_marker` transform from a
   one-time-measured mounting offset (`marker_offset_x/y/z/roll/pitch/yaw`
   parameters — see `ground_controller/launch/coordinate_bridge.launch.py`
   for the measured defaults, including a **-90° yaw** correction found by
   placing a target directly in front of the robot and checking which axis
   of `/camera/target_pose_raw` picked up that "forward" distance — the
   marker's own frame turned out to be rotated relative to `base_link`,
   not aligned with it as first assumed).
2. Consumes the ground robot's own live localization (`map → base_link`,
   already required for `NavigateToPose` to work).
3. Transforms the raw pose into `map` frame with `tf2`, looking the
   transform up at the **latest available time** rather than at the
   detection's own stamp. Asking for the exact stamp is what that field is
   for, but it requires the stamp to still be inside tf2's ~10s buffer, and
   successful detections here are ~17.8s apart (most frames yield no
   marker+target pair) — so every lookup asked for a stamp already aged out
   and failed, and no target ever reached the scheduler. Using the latest
   transform is sound here: the robot has not been dispatched while the
   scheduler waits for a target, so `map → base_link` is not moving, and
   `base_link → aruco_marker` is static.
4. Flattens `z` to `0.0` before publishing on `/camera/target_pose` — the
   target is known to sit on the ground, so whatever the marker-relative
   backprojection computes for height just reflects marker mounting height
   plus depth-sensor noise, not a meaningful target elevation.

## Known gaps / not yet built

- **No cross-frame temporal fusion.** Each successful frame's best detection
  is used as-is; there's no averaging or consistency-voting across multiple
  successful frames. In practice, detection confidence on the current rig
  sits close to the `detection_confidence_threshold` (often 0.4-0.8), so
  successful frames arrive sparsely rather than every frame - fusion across
  frames could make this more robust, at the cost of latency. How sparse
  depends heavily on the rig: a run with the marker only intermittently in
  view measured ~17.8s between successful detections, far enough apart to
  break anything that assumes a steady stream (see `coordinate_bridge_node`
  above).
- **No multi-target support.** Only the single highest-confidence detection
  per frame is reported.
- **Tuned for a near-top-down viewing angle.** The aspect-ratio filter and
  confidence threshold were calibrated against the current benchtop rig's
  viewing angle and specific distractor object; both may need retuning if
  the camera's mounting angle, distance, or scene contents change
  significantly (e.g. moving from a fixed overhead camera to one actually
  flying on the UAV).
- **Camera requires true USB 3.0 bandwidth.** A D435/D436 streaming
  simultaneous color (1280x720) + depth needs USB3 SuperSpeed; plugged into
  a USB2 port or a USB2-only hub, the device still enumerates and the ROS
  node still starts, but frames silently never arrive
  ("Frames didn't arrived within 5 seconds" from the realsense driver) -
  check `lsusb -t` for the negotiated link speed (look for `5000M`+ next to
  the camera's `uvcvideo` entries, not `480M`) before assuming a software
  problem.

---

## Key parameters

| Parameter | Default | Notes |
|---|---|---|
| `color_image_topic` | `/camera/camera/color/image_raw` | |
| `depth_image_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | must be the aligned depth topic |
| `camera_info_topic` | `/camera/camera/color/camera_info` | |
| `marker_id` | `0` | ArUco ID (DICT_4X4_50) mounted on the ground robot |
| `marker_size` | `0.15` (m) | must match the physically printed marker's side length |
| `target_class_id` | `0` | COCO "person" |
| `detection_confidence_threshold` | `0.4` | see design table above |
| `min_aspect_ratio` / `max_aspect_ratio` | `1.0` / `5.0` | box width/height bounds |
| `min_depth` / `max_depth` | `0.1` / `2.5` (m) | reject implausible depth readings; tune to the actual camera-to-floor height |
| `marker_max_age_sec` | `2.0` | |
| `marker_frame_id` | `aruco_marker` | output `PoseStamped` frame_id |
| `model_path` | `yolov8m.pt` | traded off against `yolov8n.pt` (≈2x faster, lower confidence margin) |
| `tile_size` / `tile_overlap` | `256` / `0.4` | |
| `single_shot` | `false` | set `true` for a fixed camera that only needs one confident reading |
| `save_tile_debug_dir` | `''` (disabled) | if set, saves the single tile image containing the winning detection (with box drawn) for visual debugging |
