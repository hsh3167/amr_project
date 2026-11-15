"""python -m cad.p30 {validate,build} [--part GROUP] [--pose LEFT=5] [--format step,glb] [--fast]"""
import argparse
import json
from pathlib import Path
from ..parameters import ROOT
from .build import GROUPS, P30, build_records
from .check import validate


def main():
    ap = argparse.ArgumentParser(description='P30 profile chassis (separate variant)')
    ap.add_argument('command', choices=['validate', 'build'])
    ap.add_argument('--part', action='append', choices=GROUPS)
    ap.add_argument('--pose', action='append', default=[], help='LEFT=dz / RIGHT=dz, vertical drive travel (+-10)')
    ap.add_argument('--format', default='step,glb')
    ap.add_argument('--fast', action='store_true', help='Skip interference, swivel, support and LiDAR checks')
    ap.add_argument('--output', type=Path, default=ROOT/'cad-output/p30')
    ap.add_argument('--name', default='amr_p30')
    a = ap.parse_args()
    motion = {}
    for item in a.pose:
        k, v = item.split('='); v = float(v)
        if k not in ('LEFT', 'RIGHT') or abs(v) > 10:
            ap.error('Pose: LEFT/RIGHT within +-10 mm')
        motion[k] = v
    P = P30(motion); records = build_records(P, a.part)
    result = validate(P, records, full=not a.fast and a.part is None)
    if result['valid'] and a.command == 'build':
        from ..export import write
        fmts = () if a.format == 'none' else tuple(a.format.split(','))
        if fmts:
            result['export'] = {k: v for k, v in write(records, a.output, fmts, False, a.name).items() if k in ('files', 'step_roundtrip')}
    a.output.mkdir(parents=True, exist_ok=True)
    (a.output/(a.command+'_report.json')).write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps({'command': a.command, 'valid': result['valid'], 'components': result['components'], 'errors': result['errors'][:40]}, ensure_ascii=True))
    if not result['valid']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
