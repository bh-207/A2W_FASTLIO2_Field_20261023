"""Fail closed on wrong runtime registration libraries in isolated experiments."""
import argparse
import hashlib
import json
from pathlib import Path
import re

REQUIRED={'liba2w_registration.so','liba2w_algorithm_suite.so'}
LINE=re.compile(r'^\s*(?:(\S+)\s+=>\s+)?(/.+?)(?:\s+\(0x[0-9a-fA-F]+\))?\s*$')

def verify(text,expected_library_directory):
    expected=Path(expected_library_directory).resolve(strict=True)
    found={}; all_libraries={}
    for line in text.splitlines():
        match=LINE.match(line)
        if not match:continue
        name,path=match.groups(); file=Path(path).resolve(strict=True)
        # ldd lists explicit LD_PRELOAD entries without a "soname =>" prefix.
        # Resolve their required identity too; canonical-directory checks below
        # still reject a same-named library outside the isolated prefix.
        if name is None:
            name=Path(path).name if Path(path).name in REQUIRED else file.name
        if not file.is_file():raise ValueError('runtime library is not a file')
        record={'path':str(file),'sha256':hashlib.sha256(file.read_bytes()).hexdigest()}
        all_libraries[str(file)]=record
        if name in REQUIRED:
            if file.parent != expected:raise ValueError(f'{name}: outside isolated common library directory')
            if name in found:raise ValueError(f'{name}: duplicate runtime resolution')
            found[name]=record
    if set(found)!=REQUIRED:raise ValueError('missing required isolated libraries: '+','.join(sorted(REQUIRED-set(found))))
    return {'required':found,'all_libraries':all_libraries,'expected_directory':str(expected)}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ldd_output',type=Path);parser.add_argument('expected_library_directory',type=Path)
    parser.add_argument('new_output_json',type=Path); args=parser.parse_args()
    result=verify(args.ldd_output.read_text(),args.expected_library_directory)
    with args.new_output_json.open('x')as stream:json.dump(result,stream,indent=2)
if __name__=='__main__':main()
