import json
import threading
import time

from bt_common.macros import MacroError, expand, load_macros, load_whitelist

import rclpy
from bt_pipeline_msgs.msg import BtPlan
from bt_pipeline_msgs.srv import GenerateXml, ValidateTree
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bt_orchestrator.mode import GO_HOME, RecoveryPolicy, ask_human
from bt_orchestrator.policy import refuse_no_arm
from bt_orchestrator.task_fsm import TaskFsm, repair_prompt


class OrchestratorNode(Node):
    """bt/task_spec (JSON from llm_bridge) -> SLM -> validator, retrying with errors.

    Only a validated tree is published with valid=true; after max_attempts the last
    failing attempt is published with valid=false so consumers can refuse it.

    It also tracks the task lifecycle (task_fsm.py, published latched on task/state). With
    max_repairs > 0 a failed run is re-planned: the LLM gets the task, the failing spec and the
    failed node on bt/repair. The new plan still only runs if bt_executor allows it (auto_run).
    """

    def __init__(self):
        super().__init__('orchestrator_node')
        self.declare_parameter('max_attempts', 3)
        self.declare_parameter('call_timeout_sec', 60.0)
        self.declare_parameter('macros_file', '')     # '' = framework_bringup/param/macros.yaml
        self.declare_parameter('max_repairs', 0)      # 0 = never re-plan a failed run on its own
        self.declare_parameter('memory_enabled', False)   # WICL: retrieve Task Memories (bt_memory) before planning
        self.declare_parameter('memory_dir', '')          # '' = ~/.local/share/robocup_fw/bt_memory
        self.declare_parameter('memory_verbose', False)   # log every retrieval: scores, ids, which memory was offered
        self.declare_parameter('memory_places_file', '')   # '' = param/places.yaml; read per task (new map = no restart)
        self.declare_parameter('memory_k', 3)
        self.declare_parameter('memory_min_score', 0.5)
        self.declare_parameter('memory_embodiment', 'kobuki')
        self.declare_parameter('memory_use_candidate', True)   # False: examples only, never skip the planner
        self.declare_parameter('recovery_policy', True)   # choose the recovery per failure (mode.py); False = always re-plan
        cb = ReentrantCallbackGroup()
        self.slm = self.create_client(GenerateXml, 'bt_slm/generate', callback_group=cb)
        self.val = self.create_client(ValidateTree, 'bt_validator/validate',
                                      callback_group=cb)
        self.plan_pub = self.create_publisher(BtPlan, 'bt/plan', 10)
        self.create_subscription(String, 'bt/task_spec', self._on_spec, 10,
                                 callback_group=cb)
        self.lock = threading.Lock()   # one plan at a time
        try:
            path = self.get_parameter('macros_file').value or None
            self.macros = load_macros(path)
            self.whitelist = load_whitelist(path)     # which macros a task spec itself may call
        except (MacroError, OSError) as e:
            self.get_logger().error(f'macros not loaded ({e}): macro steps will be refused')
            self.macros, self.whitelist = {}, None
        self.fsm = TaskFsm(self.get_parameter('max_repairs').value,
                           RecoveryPolicy() if self.get_parameter('recovery_policy').value else None)
        self.run_cli = self.create_client(Trigger, 'bt_executor/run', callback_group=cb)
        self.fsm_lock = threading.Lock()
        self.state_pub = self.create_publisher(
            String, 'task/state', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                             durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.repair_pub = self.create_publisher(String, 'bt/repair', 10)
        self.create_subscription(String, 'bt/task', self._on_task, 10, callback_group=cb)
        self.memory = None
        if self.get_parameter('memory_enabled').value:
            from bt_orchestrator.memory_hook import MemoryHook
            p = self.get_parameter
            self.memory = MemoryHook(p('memory_dir').value, p('memory_embodiment').value, p('memory_k').value,
                                     p('memory_min_score').value, p('memory_use_candidate').value,
                                     places=self._places)
            self.scene = {}
            self.ctx_pub = self.create_publisher(String, 'bt/task_ctx', 10)   # the bridge listens here (see pipeline.launch.py)
            self.create_subscription(String, 'scene/summary', self._on_scene, 1, callback_group=cb)
        self.create_subscription(String, 'bt/status', self._on_status, 10, callback_group=cb)
        self.create_subscription(String, 'llm/error', self._on_llm_error, 10, callback_group=cb)
        self._publish_state()

    def _publish_state(self):
        self.state_pub.publish(String(data=json.dumps(self.fsm.snapshot())))

    def _places(self):
        """Names in the current places file; raises (-> memory skipped, planning continues) if it cannot be read."""
        import os

        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = self.get_parameter('memory_places_file').value or os.path.join(
            get_package_share_directory('framework_bringup'), 'param', 'places.yaml')
        with open(path) as f:
            return set((yaml.safe_load(f) or {}).get('places') or {})

    def _on_scene(self, msg):
        try:
            self.scene = json.loads(msg.data)
        except ValueError:
            pass

    def _on_task(self, msg):
        with self.fsm_lock:
            self.fsm.on_task(msg.data)
            self._publish_state()
        if self.memory:
            self._plan_from_memory(msg.data)

    def _plan_from_memory(self, text):
        """Retrieve memories: a candidate that validates NOW becomes the plan (the planner is skipped);
        otherwise the bridge gets the task plus few-shot examples. Memory trouble never blocks planning."""
        try:
            cand, planner_text = self.memory.on_task(
                text, self.scene, lambda xml: list(self._call(self.val, ValidateTree.Request(xml=xml)).errors))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'task memory unavailable, planning normally: {type(e).__name__}: {e}')
            cand, planner_text = None, text
        if self.get_parameter('memory_verbose').value:
            self.get_logger().info('task memory: ' + ('; '.join(self.memory.report) or 'nothing retrieved'))
        if cand:
            plan = BtPlan(task=text, xml=cand.xml, valid=True, attempts=0)
            plan.header.stamp = self.get_clock().now().to_msg()
            self.plan_pub.publish(plan)
            with self.fsm_lock:
                self.fsm.on_plan(True)
                self._publish_state()
            self.get_logger().info(f'plan from task memory {cand.id} (re-validated), planner skipped')
        else:
            self.ctx_pub.publish(String(data=planner_text))

    def _on_llm_error(self, msg):
        """The bridge got no usable reply: answer the task with an invalid plan instead of staying silent."""
        try:
            err = str(json.loads(msg.data).get('error', 'unknown error'))
        except ValueError:
            err = msg.data
        plan = BtPlan(valid=False, errors=[f'no plan from the LLM: {err}'])
        plan.header.stamp = self.get_clock().now().to_msg()
        self.plan_pub.publish(plan)
        with self.fsm_lock:
            self.fsm.on_plan(False, plan.errors)
            self._publish_state()
        self.get_logger().error(plan.errors[0])

    def _on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        with self.fsm_lock:
            repair = self.fsm.on_status(st.get('state', ''), st.get('message', ''),
                                        st.get('failed_node', ''))
            self._publish_state()
        if repair:
            self._recover(repair)

    def _recover(self, r):
        """Carry out the recovery policy's decision. go_home / ask_human use fixed specs: no LLM involved."""
        where = r['failed_node'] or '?'
        self.get_logger().warn(f"run failed at {where}: recovery {r['attempt']}/{r['of']} = {r['action']}")
        if r['action'] == 'replan':
            self.repair_pub.publish(String(data=repair_prompt(r)))
        elif r['action'] == 'retry':
            self._start_executor()
        else:
            spec = GO_HOME if r['action'] == 'go_home' else ask_human(f"{r['result']} at {where}")
            self._on_spec(String(data=json.dumps(spec)))
            with self.fsm_lock:
                ready = self.fsm.state == 'ready'
            if ready:
                self._start_executor()

    def _start_executor(self):
        for _ in range(8):          # the executor publishes 'finished' just before it accepts a new run
            res = self._call(self.run_cli, Trigger.Request())
            if res.success or 'already running' not in res.message:
                break
            time.sleep(0.25)
        if not res.success:
            self.get_logger().error(f'executor refused the recovery run: {res.message}')

    def _call(self, client, req):
        done = threading.Event()
        fut = client.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(self.get_parameter('call_timeout_sec').value):
            fut.cancel()
            raise TimeoutError(f'{client.srv_name} timed out')
        return fut.result()

    def _on_spec(self, msg):
        spec = refuse_no_arm(msg.data) or msg.data       # no arm: say so instead of driving somewhere
        if spec != msg.data:
            self.get_logger().warn('goal needs the arm: replaced by a refusal')
        with self.fsm_lock:
            self.fsm.on_spec(spec)
        with self.lock:
            plan = BtPlan(task_spec=spec, valid=False)
            plan.header.stamp = self.get_clock().now().to_msg()
            try:
                self._run(plan, self._expanded(plan.task_spec))
            except (MacroError, ValueError) as e:        # bad JSON or a bad macro call: tell the LLM
                plan.errors = [f'task spec rejected: {e}']
                self.get_logger().warn(plan.errors[0])
            except Exception as e:  # noqa: BLE001
                plan.errors = [f'pipeline error: {type(e).__name__}: {e}']
                self.get_logger().error(plan.errors[0])
            self.plan_pub.publish(plan)
            with self.fsm_lock:
                self.fsm.on_plan(plan.valid, plan.errors)
                self._publish_state()
            self.get_logger().info(
                f'plan valid={plan.valid} attempts={plan.attempts} errors={len(plan.errors)}')

    def _expanded(self, task_spec):
        """Macro steps -> real nodes. plan.task_spec stays the short form the LLM wrote."""
        return json.dumps(expand(json.loads(task_spec), self.macros, self.whitelist))

    def _run(self, plan, expanded):
        for attempt in range(1, self.get_parameter('max_attempts').value + 1):
            plan.attempts = attempt
            g = self._call(self.slm, GenerateXml.Request(
                task_spec=expanded, previous_xml=plan.xml,
                previous_errors=list(plan.errors)))
            if not g.ok:
                plan.errors = [f'SLM failed: {g.error}']
                continue
            plan.xml = g.xml
            v = self._call(self.val, ValidateTree.Request(xml=plan.xml))
            plan.errors = list(v.errors)
            if v.valid:
                plan.valid = True
                return


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
