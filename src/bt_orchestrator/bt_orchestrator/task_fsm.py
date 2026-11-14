"""Task lifecycle across planning, running and repair. Pure logic, no ROS.

    idle -> planning -> ready -> running -> done
               |          |         |-> halted        (bt_executor/halt, emergency, timeout of the user)
               |          |         '-> repairing -> planning ... (at most max_repairs times)
               '-> failed '-> failed (executor refused)         '-> failed (no budget / no context)

Inputs are the three things the pipeline already publishes: bt/task, bt/plan, bt/status.
The only side effect it can ask for is a repair: re-plan the task, telling the LLM where the old
plan broke. max_repairs=0 (default) means never re-plan on its own.
"""
IDLE, PLANNING, READY, RUNNING, REPAIRING, DONE, FAILED, HALTED = (
    'idle', 'planning', 'ready', 'running', 'repairing', 'done', 'failed', 'halted')


class TaskFsm:
    def __init__(self, max_repairs=0, policy=None):
        self.max_repairs, self.policy = max_repairs, policy      # policy: RecoveryPolicy; None = always 'replan'
        self.state, self.repairs = IDLE, 0
        self.task, self.spec, self.detail = '', '', ''

    def snapshot(self):
        return {'state': self.state, 'repairs': self.repairs, 'max_repairs': self.max_repairs,
                'task': self.task, 'detail': self.detail}

    def on_task(self, text):
        """A new user task: the budget starts over."""
        self.task, self.spec, self.repairs, self.detail = text, '', 0, ''
        self.state = PLANNING

    def on_spec(self, spec):
        self.spec = spec

    def on_plan(self, valid, errors=()):
        """bt/plan from the orchestrator (also for the re-plan after a repair)."""
        if self.state not in (PLANNING, REPAIRING, IDLE):
            return          # a hand-written tree submitted mid-run does not change the lifecycle
        self.state = READY if valid else FAILED
        self.detail = '' if valid else '; '.join(errors)[:200]

    def on_status(self, state, message, failed_node=''):
        """bt/status from bt_executor. Returns a repair request dict, or None."""
        if state == 'running':
            self.state, self.detail = RUNNING, message
        elif state == 'finished' and self.state == RUNNING:
            if message == 'SUCCESS':
                self.state, self.detail = DONE, ''
            elif message == 'HALTED':
                self.state, self.detail = HALTED, 'halted'
            else:
                return self._failed_run(message, failed_node)
        return None

    def _failed_run(self, result, failed_node):
        used = self.repairs
        action = self.policy.decide(result, failed_node, used) if self.policy else 'replan'
        if action != 'abort' and used < self.max_repairs and self.task:
            self.repairs += 1
            self.state, self.detail = REPAIRING, f'{action} after {result} at {failed_node or "?"}'
            return {'task': self.task, 'spec': self.spec, 'failed_node': failed_node, 'result': result,
                    'attempt': self.repairs, 'of': self.max_repairs, 'action': action}
        self.state = FAILED
        self.detail = f'{result} at {failed_node or "?"}' + (
            '' if self.max_repairs else ' (repair disabled)') + (' (policy: abort)' if action == 'abort' else '')
        return None


def repair_prompt(r):
    """Text sent to the LLM as a new task. The LLM sees the normal catalog/scene prompt too."""
    where = f"at node {r['failed_node']}" if r['failed_node'] else 'at an unknown node'
    return (f"{r['task']}\n\nREPAIR {r['attempt']}/{r['of']}: the previous plan failed "
            f"({r['result']}) {where}. Previous task spec: {r['spec'] or 'unknown'}. "
            f"Output a corrected task spec for the same task: keep what worked, and add a "
            f"Fallback with an alternative or a retry around the step that failed.")
