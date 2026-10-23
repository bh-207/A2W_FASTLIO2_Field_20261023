#!/usr/bin/env python3
"""Separate opt-in entry for existing, physically unverified JT128 initial values."""
import json
from pathlib import Path
import sys

import mapping_capture as capture
from mapping_initial_conditions import authorize_existing_initials, condition_labels


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] != 'init':
        return capture.main(argv)
    parser = capture.parser()
    init = parser._subparsers._group_actions[0].choices['init']
    init.add_argument('--authorize-existing-unverified-initials', action='store_true',
                      help='Explicitly authorize R=I,T=0,offset=0,extrinsic_est_en=true; no calibration claim')
    init.add_argument('--authorization-note', required=True)
    args = parser.parse_args(argv)
    try:
        if args.base_config is None or args.interface is None:
            raise ValueError('explicit --base-config and audited --interface are required')
        if args.synthetic_test: raise ValueError('this entry is for existing real JT128 input only')
        base = args.base_config.resolve(strict=True)
        _, policy = authorize_existing_initials(capture.read_json(args.interface), base,
            capture.parameters(base), capture.sha256(base), args.authorization_note,
            authorized=args.authorize_existing_unverified_initials)
        session = capture.init_session(args.name,args.sessions_root,args.build_root,args.interface,
                                       base=base,experimental_policy=policy)
        metadata = capture.read_json(session/'session.json')
        print(json.dumps(dict(session=str(session),map_gravity_alignment=False,
                              **condition_labels(metadata)),indent=2))
        return 0
    except (ValueError,OSError,KeyError) as error:
        print(json.dumps(dict(error=str(error))),file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
