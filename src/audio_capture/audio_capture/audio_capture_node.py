import subprocess
import threading
import time
import wave

import rclpy
from audio_msgr.msg import AudioChunk
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def arecord_cmd(device, rate, channels):
    cmd = ['arecord', '-q', '-t', 'raw', '-f', 'S16_LE', '-r', str(rate), '-c', str(channels)]
    return cmd + (['-D', device] if device else [])


class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')
        for k, v in dict(source='arecord', device='', file='', loop=False, sample_rate=16000,
                         channels=1, chunk_ms=100, frame_id='microphone').items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.rate, self.ch = int(p('sample_rate')), int(p('channels'))
        self.nbytes = self.rate * self.ch * 2 * int(p('chunk_ms')) // 1000
        self.pub = self.create_publisher(AudioChunk, 'audio/raw', qos_profile_sensor_data)
        self.proc = None
        self.stop = threading.Event()
        target = {'arecord': self._run_arecord, 'file': self._run_file}.get(p('source'))
        if target is None:
            raise ValueError(f"source must be 'arecord' or 'file', got {p('source')!r}")
        self.thread = threading.Thread(target=target, daemon=True)
        self.thread.start()

    def _publish(self, data):
        msg = AudioChunk(sample_rate=self.rate, channels=self.ch, data=list(data))
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        self.pub.publish(msg)

    def _run_arecord(self):
        cmd = arecord_cmd(self.get_parameter('device').value, self.rate, self.ch)
        while not self.stop.is_set():
            try:
                self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                             stderr=subprocess.PIPE)
            except FileNotFoundError:
                self.get_logger().error('arecord not found (install alsa-utils)')
                return
            self.get_logger().info('capturing: ' + ' '.join(cmd))
            while not self.stop.is_set():
                data = self.proc.stdout.read(self.nbytes)
                if not data:
                    break
                self._publish(data)
            err = self.proc.stderr.read().decode(errors='replace').strip()
            self.proc.terminate()
            if self.stop.is_set():
                return
            self.get_logger().error(f'arecord exited: {err or "no output"}; retrying in 2 s')
            time.sleep(2.0)

    def _run_file(self):
        path = self.get_parameter('file').value
        try:
            wf = wave.open(path, 'rb')
        except (OSError, wave.Error) as e:
            self.get_logger().error(f'cannot open {path!r}: {e}')
            return
        with wf:
            if (wf.getframerate(), wf.getnchannels(), wf.getsampwidth()) != (self.rate, self.ch, 2):
                self.get_logger().error(
                    f'{path}: need {self.rate} Hz / {self.ch} ch / 16-bit, got '
                    f'{wf.getframerate()} Hz / {wf.getnchannels()} ch / {8 * wf.getsampwidth()}-bit')
                return
            frames = self.nbytes // (2 * self.ch)
            period = frames / self.rate
            while not self.stop.is_set():
                data = wf.readframes(frames)
                if not data:
                    if self.get_parameter('loop').value:
                        wf.rewind()
                        continue
                    self.get_logger().info('end of file')
                    return
                self._publish(data)
                time.sleep(period)   # real-time pacing

    def destroy_node(self):
        self.stop.set()
        if self.proc is not None:
            self.proc.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
