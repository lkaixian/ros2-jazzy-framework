from audio_msgr.srv import Speak
from bt_common.executor_node import (call_service, ExtensionExecutor, PortError, port_bool,
                                     port_str, run_executor)
from face_recognition_msgr.srv import Enroll
from std_srvs.srv import Trigger

from face_recognition.gallery import valid_name

MAX_TEXT = 300


def render_text(text, person=''):
    """Substitute {person}; an unknown/empty name becomes "there". Bounded length."""
    person = (person or '').strip()
    if person.lower() in ('', 'unknown'):
        person = 'there'
    out = text.replace('{person}', person).strip()
    if not out:
        raise PortError('text is empty')
    if len(out) > MAX_TEXT:
        raise PortError(f'text longer than {MAX_TEXT} characters')
    return out


class SocialExtension(ExtensionExecutor):
    def __init__(self):
        super().__init__('social_extension')
        for k, v in {'speak_service': 'tts/speak', 'stop_service': 'tts/stop',
                     'enroll_service': 'face/enroll', 'speak_timeout_sec': 30.0,
                     'enroll_timeout_sec': 10.0}.items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.speak = self.create_client(Speak, p('speak_service'))
        self.stop = self.create_client(Trigger, p('stop_service'))
        self.enroll = self.create_client(Enroll, p('enroll_service'))
        self.handlers = {'Speak': self._speak, 'EnrollFace': self._enroll}

    def _speak(self, ports, ctl):
        text = render_text(port_str(ports, 'text'), ports.get('person', ''))
        req = Speak.Request(text=text, voice=ports.get('voice', ''),
                            blocking=port_bool(ports, 'wait', True))
        res, err = call_service(self.speak, req, ctl, self.get_parameter('speak_timeout_sec').value)
        if res is None:
            if err in ('cancelled', 'timeout'):
                self.stop.call_async(Trigger.Request())      # silence the robot
            return False, err
        return res.ok, res.error or 'spoken'

    def _enroll(self, ports, ctl):
        name = port_str(ports, 'person')
        if not valid_name(name):
            return False, f'invalid name {name!r}'
        res, err = call_service(self.enroll, Enroll.Request(name=name), ctl,
                                self.get_parameter('enroll_timeout_sec').value)
        if res is None:
            return False, err
        return res.ok, res.message


def main(args=None):
    run_executor(SocialExtension, args)


if __name__ == '__main__':
    main()
