"""Bounded read-only colcon subprocess diagnostic; creates only cmake --version."""
import asyncio
import inspect
import os
import signal
import sys

from colcon_core.subprocess import run


def chain(coroutine):
    seen = set()
    while coroutine is not None and id(coroutine) not in seen:
        seen.add(id(coroutine))
        frame = getattr(coroutine, "cr_frame", None) or getattr(coroutine, "gi_frame", None)
        if frame:
            print(frame.f_code.co_name, frame.f_code.co_filename, frame.f_lineno, flush=True)
            target = frame.f_locals.get("self")
            if target is not None and hasattr(target, "_returncode"):
                print(" subprocess transport", "pid", getattr(target,"_pid",None),
                    "returncode",target._returncode,"pipes",getattr(target,"_pipes",None),
                    "finished",getattr(target,"_finished",None),flush=True)
        coroutine = getattr(coroutine, "cr_await", None) or getattr(coroutine, "gi_yieldfrom", None)


async def main():
    print("pid", os.getpid(), "SIGCHLD", signal.getsignal(signal.SIGCHLD),
          "watcher", type(asyncio.get_child_watcher()).__name__, flush=True)

    def expired():
        print("WATCHDOG: colcon subprocess failed to finish in 3 seconds", flush=True)
        watcher=asyncio.get_child_watcher()
        print("watcher pending threads",list(getattr(watcher,"_threads",{})),flush=True)
        for task in asyncio.all_tasks():
            print("TASK", task.get_name(), flush=True)
            chain(task.get_coro())
        os._exit(90)

    watchdog = asyncio.get_running_loop().call_later(3, expired)
    result = await run(["/usr/bin/cmake", "--version"], lambda row: print("out", repr(row), flush=True),
                       lambda row: print("err", repr(row), flush=True), use_pty="--pty" in sys.argv, capture_output=True)
    watchdog.cancel()
    print("returncode", result.returncode, flush=True)


if __name__ == "__main__":
    asyncio.run(main())
