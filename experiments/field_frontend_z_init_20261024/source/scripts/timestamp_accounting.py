#!/usr/bin/env python3
"""Exact post-publication-intake-policy-backend-Bundle accounting, offline only."""
import argparse
from bisect import bisect_left
from collections import Counter,defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import re


def integer(value):
    if isinstance(value,bool)or not(isinstance(value,int)or isinstance(value,str)and re.fullmatch(r'[+-]?\d+',value)):
        raise ValueError('exact integer required; float-rounded ns forbidden')
    return int(value)


def account(expected,observed,tolerance=0):
    expected=[integer(x)for x in expected];observed=[integer(x)for x in observed]
    ordered=sorted(set(expected));mapped=[];errors=[];unexpected=[]
    for stamp in observed:
        if not tolerance:
            mapped.append(stamp);errors.append(0);continue
        lo=bisect_left(ordered,stamp-tolerance);hi=bisect_left(ordered,stamp+tolerance+1)
        options=ordered[lo:hi]
        if len(options)>1:raise ValueError('ambiguous derived timestamp association')
        if not options:unexpected.append(stamp);continue
        mapped.append(options[0]);errors.append(abs(stamp-options[0]))
    need,have=Counter(expected),Counter(mapped)
    missing=list((need-have).elements());extra=list((have-need).elements())+unexpected
    return {'expected_count':len(expected),'observed_count':len(observed),'matched_count':sum((need&have).values()),
            'missing_count':len(missing),'unexpected_count':len(extra),'missing_stamps_ns':missing,'unexpected_stamps_ns':extra,
            'complete':not missing and not extra,'earliest_expected_ns':min(expected,default=None),
            'earliest_observed_ns':min(observed,default=None),'tolerance_ns':tolerance,'max_stamp_error_ns':max(errors,default=None)}


def csv_rows(path):
    with Path(path).open(newline='')as f:
        reader=csv.DictReader(f);rows=list(reader)
        if any(None in row or any(v is None for v in row.values())for row in rows):raise ValueError('truncated CSV: '+str(path))
        return rows


