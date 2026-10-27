"""Verify hashes and restore selected samples. Does not run ROS or devices."""
from pathlib import Path
import argparse,gzip,hashlib,json,tarfile

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    source=Path(__file__).resolve().parent
    for line in (source/'SHA256SUMS').read_text().splitlines():
        expected,name=line.split('  ',1);path=source/name
        if not path.resolve().is_relative_to(source):raise ValueError('unsafe manifest path')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise ValueError('hash mismatch: '+name)
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    bags=out/'bags';bags.mkdir();maps=out/'maps';maps.mkdir()
    for archive in sorted((source/'bags').glob('*.tar.gz')):
        with tarfile.open(archive,'r:gz')as tar:
            members=tar.getmembers()
            for member in members:
                if not member.isfile()or member.issym()or member.islnk():raise ValueError('non-file archive entry')
                path=(bags/member.name).resolve()
                if not path.is_relative_to(bags):raise ValueError('unsafe archive path')
            for member in members:
                path=bags/member.name;path.parent.mkdir(parents=True,exist_ok=True)
                with tar.extractfile(member)as f,path.open('xb')as dest:dest.write(f.read())
    for packed in sorted((source/'maps').glob('*.gz')):
        with gzip.open(packed,'rb')as f,(maps/packed.stem).open('xb')as dest:dest.write(f.read())
    print(json.dumps(dict(output=str(out),bags=sorted(p.name for p in bags.iterdir()),maps=sorted(p.name for p in maps.iterdir()),complete_original_bag_backup=False),indent=2))

if __name__=='__main__':main()
