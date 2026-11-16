"""ROS service front end: wicl/rehearse (RehearseTask), wicl/memories (ListMemories).
Planner and reflector go through llm_bridge's llm/query service, so failover and keys are the bridge's job."""
import threading

import rclpy
from bt_memory.schema import MemoryStore
from bt_orchestrator.memory_hook import DEFAULT_DIR
from bt_pipeline_msgs.srv import ListMemories, LlmQuery, RehearseTask
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from bt_rehearsal.llm import LlmPlanner, Reflector
from bt_rehearsal.loop import rehearse, to_memory
from bt_rehearsal.sim import Task
from bt_rehearsal.wiring import FRAMEWORK_VERSION, make_env


class RehearsalNode(Node):
    def __init__(self):
        super().__init__('rehearsal_node')
        for k, v in dict(memory_dir='', embodiment='kobuki', max_attempts=4, timeout_sec=120.0,
                         model_version='llm', llm_timeout_sec=60.0).items():
            self.declare_parameter(k, v)
        cb = ReentrantCallbackGroup()
        self.llm = self.create_client(LlmQuery, 'llm/query', callback_group=cb)
        self.store = MemoryStore(self.get_parameter('memory_dir').value or DEFAULT_DIR)
        self.env = make_env()
        self.lock = threading.Lock()      # one rehearsal at a time: the sim is stateful
        self.create_service(RehearseTask, 'wicl/rehearse', self._rehearse, callback_group=cb)
        self.create_service(ListMemories, 'wicl/memories', self._list, callback_group=cb)

    def _chat(self, system, prompt):
        if not self.llm.wait_for_service(timeout_sec=1.0):
            raise ConnectionError('llm/query not available')
        done, req = threading.Event(), LlmQuery.Request(system=system, prompt=prompt, json_mode=False)
        fut = self.llm.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(self.get_parameter('llm_timeout_sec').value):
            fut.cancel()
            raise TimeoutError('llm/query timed out')
        r = fut.result()
        if not r.ok:
            raise ConnectionError(r.error)
        return r.text

    def _rehearse(self, req, res):
        p = self.get_parameter
        with self.lock:
            try:
                task = Task(req.task, req.goal_place or None)
                out = rehearse(task, LlmPlanner(self._chat, self.env['listing']), self.env['build'],
                               self.env['validate'], self.env['sim'], Reflector(self._chat) if req.reflection else None,
                               req.max_attempts or p('max_attempts').value, req.timeout_sec or p('timeout_sec').value)
                res.status, res.attempts, res.lessons = out.status, out.attempts, out.lessons
                if out.success:
                    m = to_memory(task, out, p('embodiment').value, FRAMEWORK_VERSION, p('model_version').value)
                    self.store.save(m)
                    res.ok, res.memory_id = True, m.id
            except Exception as e:  # noqa: BLE001
                res.status, res.error = 'error', f'{type(e).__name__}: {e}'
        return res

    def _list(self, req, res):
        mems, res.problems = self.store.all()
        res.ids = [m.id for m in mems]
        res.summaries = [f'{m.embodiment}: {m.task}' for m in mems]
        res.json = next((m.to_json() for m in mems if m.id == req.memory_id), '')
        return res


def main(args=None):
    rclpy.init(args=args)
    node = RehearsalNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
