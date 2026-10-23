#!/usr/bin/env python3
"""Read-only integer timestamp multiplicities, in both directions.

Only extracts fixed CDR Header bytes with SQLite substr; large cloud payloads
are not decoded or modified. Nonempty WAL is refused rather than ignored.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import sqlite3
import struct

def headers(bag,topic):
    result=Counter()
    for path in sorted(Path(bag).glob('*.db3')):
        wal=Path(str(path)+'-wal')
        if wal.exists() and wal.stat().st_size:
            raise ValueError('freeze/close Bag before immutable accounting: '+str(wal))
        with sqlite3.connect('file:'+str(path.resolve())+'?mode=ro&immutable=1',uri=True) as database:
            for data, in database.execute('SELECT substr(data,1,12) FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name=?)',(topic,)):
                if len(data)!=12 or data[:2] not in [b'\x00\x01',b'\x00\x00']:
                    raise ValueError('unsupported CDR encapsulation')
                sec,ns=struct.unpack(('<' if data[1]==1 else '>')+'iI',data[4:12])
                if ns>=1_000_000_000: raise ValueError('invalid source integer stamp')
                result[sec*1_000_000_000+ns]+=1
    return result

def compare(a,b):
    onlya=a-b;onlyb=b-a
    def entries(counter):return [dict(stamp_ns=t,multiplicity=n) for t,n in sorted(counter.items())]
    return dict(a_count=sum(a.values()),b_count=sum(b.values()),matched_count=sum((a&b).values()),
        a_only_count=sum(onlya.values()),b_only_count=sum(onlyb.values()),
        a_only=entries(onlya),b_only=entries(onlyb),complete=bool(a) and bool(b) and not onlya and not onlyb)

def run(session,original=None):
    session=Path(session);callback=Counter()
    with (session/'input.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['status']=='adapted':callback[int(row['raw_stamp_ns'])]+=1
    recorded=headers(session/'bag','/lidar_imu');first=min(callback) if callback else None;last=max(callback) if callback else None
    interval=lambda c:Counter({t:n for t,n in c.items() if first is not None and first<=t<=last})
    result=dict(schema='bidirectional_integer_input_accounting_v1',callback_interval=[first,last],
        callback_vs_recorded=compare(callback,interval(recorded)),
        callback_vs_recorded_full=compare(callback,recorded),
        labels=dict(a='frontend adapted callback',b='same-session recorded Bag'),
        scope='Exact header integer timestamp multiplicities; no upstream driver loss claim, no subtraction to a net loss.')
    if (session/'published_raw.csv').is_file():
        with (session/'published_raw.csv').open() as stream:
            published=Counter(int(r['header_ns']) for r in csv.DictReader(stream) if r['topic']=='/lidar_imu')
        result['published_vs_callback']=compare(interval(published),callback)
        result['published_vs_callback_full']=compare(published,callback)
        result['published_vs_recorded_full']=compare(published,recorded)
    if original:
        source=headers(original,'/lidar_imu')
        result['original_bag_vs_callback']=compare(interval(source),callback)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('output',type=Path);p.add_argument('--original',type=Path)
    a=p.parse_args();result=run(a.session,a.original)
    with a.output.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['callback_vs_recorded','published_vs_callback','published_vs_recorded_full','original_bag_vs_callback']},indent=2))

if __name__=='__main__':main()
