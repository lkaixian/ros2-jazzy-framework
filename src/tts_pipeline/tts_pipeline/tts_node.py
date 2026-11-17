import queue
import subprocess
import threading
import time

import rclpy
from audio_msgr.msg import TtsStatus
from audio_msgr.srv import Speak
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from tts_pipeline import backends


class TtsNode(Node):
    """tts/say (String) and tts/speak (service) -> queue -> engine; status on tts/status."""

    def __init__(self):
        super().__init__('tts_node')
        for k, v in dict(engine='espeak', voice='', rate=160, piper_model='',
                         piper_rate=22050, audio_device='', max_queue=16,
                             kokoro_model='', kokoro_voices='', kokoro_speed=1.0,
                             kokoro_lang='en-us').items():
            self.declare_parameter(k, v)
        self.engine = self.get_parameter('engine').value
        if self.engine not in ('espeak', 'piper', 'kokoro', 'mock'):
            raise ValueError(f'unknown engine {self.engine!r}')
        self.q = queue.Queue(maxsize=self.get_parameter('max_queue').value)
        self.proc = None
        self.lock = threading.Lock()
        self.status = self.create_publisher(TtsStatus, 'tts/status', 10)
        cb = ReentrantCallbackGroup()
        self.create_subscription(String, 'tts/say', lambda m: self._enqueue(m.data, '', None),
                                 10, callback_group=cb)
        self.create_service(Speak, 'tts/speak', self._on_speak, callback_group=cb)
        self.create_service(Trigger, 'tts/stop', self._on_stop, callback_group=cb)
        threading.Thread(target=self._worker, daemon=True).start()
        self._publish(TtsStatus.IDLE)
        self.get_logger().info(f'TTS engine: {self.engine}')

    def _publish(self, state, text='', error=''):
        m = TtsStatus(state=state, text=text, error=error, queued=self.q.qsize())
        m.header.stamp = self.get_clock().now().to_msg()
        self.status.publish(m)

    def _enqueue(self, text, voice, done):
        text = text.strip()
        if not text:
            return False, 'empty text'
        try:
            self.q.put_nowait((text, voice, done))
        except queue.Full:
            return False, 'TTS queue full'
        return True, ''

    def _on_speak(self, req, res):
        done = threading.Event() if req.blocking else None
        res.ok, res.error = self._enqueue(req.text, req.voice, done)
        if res.ok and done is not None:
            done.wait(120.0)
        return res

    def _on_stop(self, _req, res):
        while True:   # drop queued, then interrupt the current utterance
            try:
                self.q.get_nowait()
            except queue.Empty:
                break
        with self.lock:
            if self.proc and self.proc.poll() is None:
                self.proc.terminate()
        res.success = True
        return res

    def _launch(self, text, voice):
        p = self.get_parameter
        if self.engine == 'mock':
            return None
        if self.engine == 'espeak':
            return subprocess.Popen(backends.espeak_cmd(text, voice or p('voice').value,
                                                        p('rate').value),
                                    stderr=subprocess.PIPE)
        cmd = backends.piper_cmd(p('piper_model').value, p('piper_rate').value,
                                 p('audio_device').value)
        proc = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        proc.stdin.write(text.encode() + b'\n')
        proc.stdin.close()
        proc.stdin = None      # else communicate() flushes the closed pipe: ValueError "flush of closed file"
        return proc

    def _worker(self):
        while rclpy.ok():
            text, voice, done = self.q.get()
            self._publish(TtsStatus.SPEAKING, text)
            err = ''
            try:
                with self.lock:
                    self.proc = self._launch(text, voice)
                if self.proc is None:
                    time.sleep(0.2)
                else:
                    _, stderr = self.proc.communicate()
                    # SIGTERM from tts/stop (-15) is not an error
                    if self.proc.returncode not in (0, -15):
                        err = (stderr or b'').decode(errors='replace').strip() or \
                            f'exit code {self.proc.returncode}'
            except Exception as e:  # noqa: BLE001
                err = f'{type(e).__name__}: {e}'
            if err:
                self.get_logger().error(err)
                self._publish(TtsStatus.ERROR, text, err)
            self._publish(TtsStatus.IDLE)
            if done is not None:
                done.set()


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
