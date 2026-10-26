"""Explicit EOF barrier; completion and input integrity remain separate."""
import json
import time
from pathlib import Path

def classify(status,expected,now_ns):
    fresh=0<=now_ns-status.get('steady_ns',0)<=2_500_000_000
    ledger_ok=status.get('trace_overflow',1)==0 and status.get('status_write_errors',1)==0
    drained=(fresh and ledger_ok and status.get('computing') is False and status.get('lidar_buffer_size')==0 and
             status.get('cloud_buffer_push')==status.get('cloud_sync_consumed'))
    exact=(status.get('imu_raw_callbacks')==expected.get('/lidar_imu') and status.get('cloud_raw_callbacks')==expected.get('/lidar_points'))
    return dict(queues_drained=drained,callback_counts_exact=exact,fresh=fresh,ledger_ok=ledger_ok,
                imu_tail_remaining=status.get('imu_buffer_size'),status=status)

def wait_frontend(session,expected,timeout=120):
    end=time.monotonic()+timeout;stable_since=None;previous=None;samples=[]
    while time.monotonic()<end:
        now=time.monotonic_ns()
        try:status=json.loads((Path(session)/'frontend_processing_status.json').read_text())
        except (OSError,ValueError):time.sleep(.2);continue
        current=classify(status,expected,now)
        signature=tuple(status.get(k) for k in ['imu_raw_callbacks','cloud_raw_callbacks','cloud_sync_consumed','post_count','lidar_buffer_size'])
        version=status.get('steady_ns',0)
        if current['queues_drained']:
            if signature!=previous or stable_since is None:stable_since=version
        else:stable_since=None
        previous=signature;samples.append(dict(steady_ns=now,queues_drained=current['queues_drained'],callback_counts_exact=current['callback_counts_exact'],signature=signature))
        # Advance on producer snapshots, not repeated reads of a cached false
        # status while the single-thread executor is blocked in long compute.
        if stable_since is not None and version-stable_since>=2_000_000_000:
            return dict(barrier_passed=True,**current,polls=samples,
                scope='Cloud queues stable and empty; missing callbacks still FAIL. Buffered IMU tail separately accounted after close.')
        time.sleep(.2)
    return dict(barrier_passed=False,last_status=locals().get('current'),polls=samples,reason='frontend queue/status did not reach stable EOF barrier; no expected count was reduced')
