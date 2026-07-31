import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from custom_msgs.msg import TaskCommand, TaskItem

from typing import TypedDict, List, Literal
from pydantic import BaseModel
from langgraph.graph import StateGraph, END
from langchain_google_genai import ChatGoogleGenerativeAI

class IntentResult(BaseModel):
    intent: str
    goal_zone: str

class PlannedTask(BaseModel):
    task: Literal["aerial_recon", "map_injection", "navigate_to_target",
                  "expand_search", "request_backup", "return_to_base"]
    agent: Literal["uav", "scheduler", "ground"]
    reason: str  # LLM 说明选择该任务的依据

class TaskPlan(BaseModel):
    tasks: List[PlannedTask]

class PlanState(TypedDict):
    command: str
    intent: str
    goal_zone: str
    tasks: List[dict]
    feedback: str

llm = ChatGoogleGenerativeAI(model="gemini-2.5-flash")
llm_intent = llm.with_structured_output(IntentResult)
llm_plan = llm.with_structured_output(TaskPlan)

def parse_intent(state: PlanState) -> PlanState:
    result: IntentResult = llm_intent.invoke([
        # The intent is what decompose_tasks reasons over, so whatever gets
        # dropped here is effectively invisible to planning. Observed live:
        # "Go to Zone B to search trapped person" was reduced to "search for
        # trapped person" - the navigation verb was gone, only the purpose
        # survived, and the planner then produced a recon-only sequence.
        # intent 是 decompose_tasks 用来推理的输入,所以在这一步被丢掉的东西,
        # 对规划来说等于不存在。实测过:"Go to Zone B to search trapped person"
        # 被压成了 "search for trapped person"——导航动词没了,只剩下目的,
        # 于是规划器只排了建图这一步。
        {"role": "system", "content": (
            "Extract the mission intent and target zone from the operator command. "
            "Keep the action the operator asked for, not only the purpose behind it: "
            "if they said to go to, reach, approach or inspect somewhere, that must "
            "survive into the intent. "
            "goal_zone can be empty if not specified."
        )},
        {"role": "user", "content": state["command"]}
    ])
    return {**state, "intent": result.intent, "goal_zone": result.goal_zone}

def decompose_tasks(state: PlanState) -> PlanState:
    # 为每个角色描述其能力和适用条件，让 LLM 自主判断哪些任务是必要的
    # Each agent is described by capability + when to use, so the LLM reasons rather than fills a template
    result: TaskPlan = llm_plan.invoke([
        {"role": "system", "content": """You are a mission planner for a search and rescue robot team.

You are writing a plan BEFORE any of it runs. Judge each capability by what
the mission will need, not by what is already true: a target the camera has
not detected yet, or a map that has not been built yet, is a reason to plan
the step that consumes it, not a reason to leave that step out.

Available capabilities — only include tasks the mission actually requires:

  aerial_recon (agent: uav)
    UAV flies over the target area to build an occupancy map.
    Use when the area layout is unknown and a map must be obtained first.

  map_injection (agent: scheduler)
    Converts the UAV map and injects it into the ground robot's navigation stack.
    Include whenever aerial_recon is part of this same plan - the ground robot
    cannot navigate on a map that was never injected.

  navigate_to_target (agent: ground)
    Ground robot drives to the target's position once the target has been
    located. Include whenever the mission requires physically reaching,
    approaching or inspecting a place or a person. The target does not have to
    be known yet - locating it happens while the plan is carried out.

  expand_search (agent: uav)
    Extends UAV reconnaissance to neighbouring zones.
    Use only when initial recon confidence is insufficient or the operator
    explicitly requests a wider search. Do NOT use for standard missions.

  request_backup (agent: scheduler)
    Dispatches a human-backup alert to the operations centre.
    Use only when the mission is assessed as high-risk, the environment is
    too complex for autonomous operation, or the operator requests human support.
    Do NOT use for routine autonomous missions.

  return_to_base (agent: uav)
    Commands the UAV to return to its launch point.
    Use for return or extraction commands, or when battery is critically low.
    Do NOT include in forward search-and-rescue missions.

Rules:
- Select only the tasks that are genuinely needed for this mission.
- Preserve dependency order: aerial_recon must precede map_injection,
  map_injection must precede navigate_to_target if both are present.
- If the command asks to go to, reach, approach, search or inspect a place -
  or to find someone there - the plan MUST end with navigate_to_target.
  Mapping an area is not the same as going to it: a plan that only surveys
  the area has not carried out such a command.
- Only leave navigate_to_target out when the command asks purely for a map or
  a survey, with nothing to be reached afterwards.
- If a map is already available, skip aerial_recon and map_injection.
- Weigh "Original command" as heavily as "Intent": the intent is a short
  summary and may have lost the action the operator actually asked for.
- For each task, provide a brief reason explaining why it is needed.
- Before answering, re-read your own reasons: if a reason justifies a step by
  what a later step needs, that later step belongs in the plan too."""},
        {"role": "user", "content": (
            f"Intent: {state['intent']}\n"
            f"Goal zone: {state['goal_zone']}\n"
            f"Original command: {state['command']}"
        )}
    ])
    return {**state, "tasks": [t.model_dump() for t in result.tasks]}

def handle_feedback(state: PlanState) -> PlanState:
    if not state.get("feedback"):
        return state
    result: TaskPlan = llm_plan.invoke([
        {"role": "system", "content": (
            "You are a mission planner. Revise the remaining task list based on "
            "execution feedback. Keep only tasks that are still necessary. "
            "Use the same task vocabulary and provide a reason for each retained or new task."
        )},
        {"role": "user", "content": (
            f"Current tasks: {state['tasks']}\n"
            f"Feedback: {state['feedback']}"
        )}
    ])
    return {**state, "tasks": [t.model_dump() for t in result.tasks], "feedback": ""}

def should_replan(state: PlanState) -> str:
    return "replan" if state.get("feedback") else "done"

graph = StateGraph(PlanState)
graph.add_node("parse_intent", parse_intent)
graph.add_node("decompose_tasks", decompose_tasks)
graph.add_node("handle_feedback", handle_feedback)
graph.set_entry_point("parse_intent")
graph.add_edge("parse_intent", "decompose_tasks")
graph.add_edge("decompose_tasks", "handle_feedback")
graph.add_conditional_edges(
    "handle_feedback",
    should_replan,
    {"replan": "decompose_tasks", "done": END}
)
# 在模块级别编译一次，node 每次收到指令直接调用 invoke 即可
planner_app = graph.compile()

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self._last_command = ""

        self.create_subscription(String, '/operator/command', self.command_callback, 10)
        self.create_subscription(String, '/planner/feedback', self._on_feedback, 10)

        self.pub = self.create_publisher(TaskCommand, '/scheduler/task_command', 10)
        # TRANSIENT_LOCAL so a dashboard connecting after the plan was
        # published still sees the last plan, not just future ones.
        # TRANSIENT_LOCAL是为了让在方案发布之后才连上的仪表盘,也能看到
        # 最后一次的方案,而不是只能看到之后新发的。
        self.pub_plan = self.create_publisher(
            String, '/planner/last_plan',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))

        self.get_logger().info('AI Planning Layer started. Waiting for operator command...')

    def command_callback(self, msg: String):
        self.get_logger().info(f'Command received: {msg.data}')
        self._last_command = msg.data
        self._invoke_and_publish(msg.data, feedback="")

    def _on_feedback(self, msg: String):
        if not self._last_command:
            self.get_logger().warn('Feedback received but no previous command to replan.')
            return
        self.get_logger().info(f'Feedback received: {msg.data}. Replanning...')
        self._invoke_and_publish(self._last_command, feedback=msg.data)

    def _invoke_and_publish(self, command: str, feedback: str):
        try:
            result = planner_app.invoke({
                "command": command,
                "intent": "",
                "goal_zone": "",
                "tasks": [],
                "feedback": feedback
            })
        except Exception as e:
            self.get_logger().error(f'Gemini API call failed. Node continues: {e}')
            return

        task_cmd = TaskCommand()
        task_cmd.intent = result["intent"]
        task_cmd.goal_zone = result.get("goal_zone", "")

        for t in result["tasks"]:
            item = TaskItem()
            item.task = t["task"]
            item.agent = t["agent"]
            task_cmd.sequence.append(item)

        self.pub.publish(task_cmd)
        self.pub_plan.publish(String(data=json.dumps({
            'command': command,
            'intent': result['intent'],
            'goal_zone': result.get('goal_zone', ''),
            'tasks': result['tasks'],  # each has task/agent/reason
        })))
        self.get_logger().info(
            f'Task command published: intent={task_cmd.intent}, '
            f'goal_zone={task_cmd.goal_zone}, {len(task_cmd.sequence)} task(s).')
        for i, item in enumerate(task_cmd.sequence, start=1):
            self.get_logger().info(
                f'  Task {i}/{len(task_cmd.sequence)}: [{item.task}] -> [{item.agent}]'
                f' | reason: {result["tasks"][i-1].get("reason", "")}')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
