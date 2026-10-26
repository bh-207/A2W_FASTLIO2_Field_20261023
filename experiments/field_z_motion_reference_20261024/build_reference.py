"""Build the isolated non-ROS inertial integration helper, no system install."""
from pathlib import Path
import hashlib,json,subprocess
E=Path(__file__).resolve().parent;B=E/'build';B.mkdir(exist_ok=True)
source=E/'inertial_reference.cpp';library=B/'libinertial_reference.so'
command=['g++','-O3','-DNDEBUG','-std=c++17','-fPIC','-shared','-I/usr/include/eigen3',str(source),'-o',str(library)]
subprocess.run(command,check=True)
(B/'identity.json').write_text(json.dumps(dict(command=command,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),binary_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),compiler=subprocess.check_output(['g++','--version'],text=True).splitlines()[0]),indent=2))
print(library)
