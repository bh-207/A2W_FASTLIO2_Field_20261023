#!/usr/bin/env python3
"""Keep real wait results even when a child exits before /proc exec observation."""
import errno
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from mapping_capture_lib import process_identity, write_json


def error_details(error):
    return dict(type=type(error).__name__, errno=getattr(error, 'errno', None), message=str(error))


def main():
    inherited_sigchld = signal.getsignal(signal.SIGCHLD)
    signal.signal(signal.SIGCHLD, signal.SIG_DFL)
    session = Path(sys.argv[1]).resolve()
    role = sys.argv[2]
    if not role.replace('_', '').isalnum():
        raise ValueError('invalid role')
    command = sys.argv[3:]
    directory = session / 'process_exits'
    directory.mkdir(exist_ok=True)
    status_path = directory / (role + '.json')
    failed_path = directory / (role + '.startup_failed.json')
    child = None
    descriptor = None
    identity = None
    observed = False
    pending_signal = None
    startup_error = None
    spawn_error = None
    cleanup_signal = None
    cleanup_signal_error = None
    wait_error = None
    code = None
    stream = None
    log_flushed = False
    log_closed = False
    log_error = None

    def forward(number, _frame):
        nonlocal pending_signal
        pending_signal = number
        # Normal signal forwarding requires observed exec identity. The fd is
        # bound to our unreaped Popen child and can never target a reused PID.
        if observed and descriptor is not None:
            try:
                signal.pidfd_send_signal(descriptor, number)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    try:
        stream = (session / 'logs' / (role + '.log')).open('x')
        try:
            child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        except (OSError, ValueError) as error:
            spawn_error = error_details(error)
        if child is not None:
            try:
                # This MUST precede every poll()/wait(): with SIGCHLD_DFL an
                # exited child remains unreaped, so pidfd_open cannot bind reuse.
                descriptor = os.pidfd_open(child.pid)
                deadline = time.monotonic() + 3
                observation_error = None
                while True:
                    try:
                        candidate = process_identity(child.pid)
                        if candidate.get('cmdline') == command:
                            identity = candidate
                            identity.update(owner_session=str(session), role=role,
                                supervisor_pid=os.getpid(), exit_status=str(status_path))
                            observed = True
                            if child.poll() is not None:
                                startup_error = 'child exited before startup registration'
                            else:
                                write_json(directory / (role + '.started.json'), identity, exclusive=True)
                                if pending_signal is not None:
                                    forward(pending_signal, None)
                            break
                    except (OSError, ValueError) as error:
                        observation_error = str(error)
                    if child.poll() is not None:
                        startup_error = 'child exited before exec identity observation'
                        break
                    if time.monotonic() >= deadline:
                        startup_error = 'exec identity observation timeout'
                        if observation_error:
                            startup_error += ': ' + observation_error
                        break
                    time.sleep(.005)
            except Exception as error:
                startup_error = 'startup observation failed: ' + str(error)
            if startup_error:
                # A startup failure never registers an unverified identity. Only
                # this immediately captured kernel-owned child fd permits cleanup.
                if descriptor is not None and child.returncode is None:
                    try:
                        signal.pidfd_send_signal(descriptor, signal.SIGTERM)
                        cleanup_signal = int(signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    except OSError as error:
                        cleanup_signal_error = error_details(error)
                try:
                    write_json(failed_path, dict(schema='owned_process_startup_failure_v1',
                        owner_session=str(session), role=role, requested_command=command,
                        supervisor_pid=os.getpid(), child_pid=child.pid,
                        identity=identity, exec_identity_observed=observed,
                        startup_error=startup_error, exit_status=str(status_path),
                        child_reap_pending=child.returncode is None,
                        child_pidfd_captured=descriptor is not None,
                        cleanup_signal=cleanup_signal, cleanup_signal_error=cleanup_signal_error), exclusive=True)
                except OSError as error:
                    print('startup failure evidence write failed: ' + str(error), file=sys.stderr)
            # Includes fast exit, proc failure, startup timeout and failed
            # registration. If TERM is ignored, retain this waiter and advertise
            # pending status above; the CLI may fail quickly without orphaning it.
            try:
                code = child.wait()
            except Exception as error:
                wait_error = error_details(error)
    except OSError as error:
        if child is None:
            spawn_error = error_details(error)
        else:
            startup_error = startup_error or 'supervisor I/O failure: ' + str(error)
            try:
                code = child.wait()
            except Exception as wait_failure:
                wait_error = error_details(wait_failure)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if stream is not None:
            try:
                stream.flush()
                os.fsync(stream.fileno())
                log_flushed = True
            except OSError as error:
                log_error = error_details(error)
            try:
                stream.close()
                log_closed = True
            except OSError as error:
                log_error = error_details(error)
    status = dict(schema='owned_process_spawn_error_v1' if child is None else 'owned_process_exit_v1',
        outcome='spawn_error' if child is None else 'child_exit',
        owner_session=str(session), role=role, requested_command=command,
        supervisor_pid=os.getpid(), child_created=child is not None,
        child_pid=child.pid if child is not None else None, identity=identity,
        exec_identity_observed=observed, startup_error=startup_error,
        spawn_error=spawn_error, cleanup_signal=cleanup_signal, cleanup_signal_error=cleanup_signal_error,
        reaped=child is not None and code is not None and wait_error is None,
        exit_code=code, wait_error=wait_error,
        exited_unix_ns=time.time_ns() if child is not None and code is not None else None,
        log_flushed=log_flushed, log_closed=log_closed, log_error=log_error,
        wait_strategy='SIGCHLD_DFL_then_pidfd_then_Popen_wait',
        inherited_sigchld_ignored=inherited_sigchld == signal.SIG_IGN)
    write_json(status_path, status, exclusive=True)
    if child is None:
        return 127 if spawn_error and spawn_error.get('errno') == errno.ENOENT else 126
    if code is None or wait_error or not log_flushed or not log_closed:
        return 2
    if startup_error and code == 0:
        return 2
    return code if code >= 0 else 128 - code


if __name__ == '__main__':
    raise SystemExit(main())
