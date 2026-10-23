"""Non-blocking single-key terminal input with reliable restoration."""

import os
import select
import termios
import tty


class TerminalKeyReader:
    def __init__(self, stream):
        self._stream = stream
        self._fd = stream.fileno()
        self._original_attributes = None

    def __enter__(self):
        if not os.isatty(self._fd):
            raise RuntimeError("waypoint_recorder must run in an interactive terminal")
        self._original_attributes = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd, termios.TCSANOW)
        return self

    def __exit__(self, _exception_type, _exception, _traceback):
        if self._original_attributes is not None:
            termios.tcsetattr(
                self._fd, termios.TCSADRAIN, self._original_attributes)
            self._original_attributes = None

    def read_available(self):
        readable, _, _ = select.select([self._fd], [], [], 0.0)
        if not readable:
            return None
        return os.read(self._fd, 1).decode("utf-8", errors="ignore")
