import os
import termios

from a2w_waypoint_tools.terminal_input import TerminalKeyReader


def test_terminal_reader_gets_a_key_without_enter_and_restores_terminal():
    master_fd, slave_fd = os.openpty()
    stream = os.fdopen(slave_fd, "rb", buffering=0)
    original = termios.tcgetattr(stream.fileno())
    try:
        with TerminalKeyReader(stream) as reader:
            os.write(master_fd, b"s")
            assert reader.read_available() == "s"
        assert termios.tcgetattr(stream.fileno()) == original
    finally:
        stream.close()
        os.close(master_fd)
