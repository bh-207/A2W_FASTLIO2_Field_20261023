"""Canonical base-anchored frontend v4 state-guard patch generator; no source edits."""
from pathlib import Path
from frontend_init_v4_runtime_patchgen import modified_sources,render_patch,once
from frontend_init_v4_state_patchgen import base

# Retain historical named configuration templates for packaging regression tests;
# modified_sources/render_patch above are the actual complete current generator.
IMU_METHODS=base.IMU_METHODS
NODE_CONFIG=base.NODE_CONFIG
if __name__=='__main__':
    print(render_patch(Path(__file__).resolve().parents[2]/'src/FAST_LIO_Hesai'),end='')
