#!/usr/bin/env python3
"""Derive exact edge-specific response evidence without changing review/geometry."""
import csv,hashlib,json,sys
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]);groups={};meta=None
for line in (root/'revocable_review.jsonl').open():
 d=json.loads(line)
 if d.get('type')=='review_hypothesis':meta=d
 elif 'records' in d and meta:
  groups.setdefault(meta['current_id'],{})[meta['mask']]=(meta,d);meta=None
out=[]
for current,hs in groups.items():
 for mask,(meta,d) in hs.items():
  edges=[]
  for bit in range(max(hs).bit_length()):
   if not mask&(1<<bit):continue
   minus=hs[mask&~(1<<bit)][1];response=[];pairs=set();same=len(d['records'])==len(minus['records'])
   if same:
    for ix,(a,b) in enumerate(zip(d['records'],minus['records'])):
     keys=['first_id','second_id','first_point_indices','second_point_indices']
     if any(a[k]!=b[k] for k in keys):same=False;break
     if abs(a['trial_signed_height_gap_m']-b['trial_signed_height_gap_m'])>1e-9 or abs(a['trial_normal_angle_deg']-b['trial_normal_angle_deg'])>1e-9:
      response.append(ix);pairs.add((a['first_id'],a['second_id']))
   edges.append(dict(bit=bit,compared_minus_mask=mask&~(1<<bit),identical_frozen_support=same,responsive_patch_count=len(response),responsive_frame_pair_count=len(pairs),responsive_record_indices=response,responsive_frame_pairs=sorted(pairs),observable=same and len(response)>=8 and len(pairs)>=2))
  out.append(dict(current_id=current,mask=mask,edges=meta['edges'],guard_verdict=d['verdict'],guard_reason=d['reason'],score_pair_mean_p95_m=meta['score_pair_mean_p95_m'],excluded_registration_ids=d['excluded_registration_ids'],selected_raw_position_min=d['selected_raw_position_min'],selected_raw_position_max=d['selected_raw_position_max'],responses=edges))
summary={'scope':'EXACT_RECORDED_PROXY_RESPONSE_NOT_PHYSICAL_TRUTH','ledger_sha256':hashlib.sha256((root/'revocable_review.jsonl').read_bytes()).hexdigest(),'hypotheses':out}
(root/'support_response_audit.json').write_text(json.dumps(summary,indent=2))
