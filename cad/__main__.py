import argparse
import json
from pathlib import Path
from .parameters import ROOT,load_design,set_parameter
from .assembly import GROUPS,build_records
from .export import write
from .validation import validate,compare_baseline

def main():
    parser=argparse.ArgumentParser(description='AMR CAD: build only requested groups; exports are explicit')
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('list')
    param=sub.add_parser('param');param.add_argument('key');param.add_argument('value')
    for command in ('build','validate','compare'):
        p=sub.add_parser(command);p.add_argument('--variant',choices=['baseline','current'],default='baseline' if command=='compare' else 'current')
        p.add_argument('--part',action='append',choices=GROUPS)
        p.add_argument('--output',type=Path,default=ROOT/'output/cad')
        p.add_argument('--pose',action='append',default=[],help='Independent wheel displacement mm: LEFT,RIGHT,LF,LB,RF,RB=value')
        if command!='compare':p.add_argument('--collisions',action='store_true')
        if command=='build':
            p.add_argument('--format',default='step',help='step,stl or none')
            p.add_argument('--individual',action='store_true');p.add_argument('--name',default='amr_current')
        if command=='compare':p.add_argument('--fast',action='store_true',help='Metrics only; omit Boolean symmetric difference')
    args=parser.parse_args()
    if args.command=='list':print('\n'.join(GROUPS));return
    if args.command=='param':print(json.dumps(set_parameter(args.key,args.value)));return
    d=load_design(args.variant)
    for entry in args.pose:
        wheel,value=entry.split('=');value=float(value)
        if d.variant=='baseline' or wheel not in ('LEFT','RIGHT','LF','LB','RF','RB'):parser.error('Pose requires current variant and a valid wheel name')
        half=d.layout['suspension']['drive' if wheel in ('LEFT','RIGHT') else 'caster']['travel_mm']/2
        if not -half<=value<=half:parser.error('Pose exceeds designed stroke')
        d.motion[wheel]=value
    records=build_records(d,args.part)
    if args.command=='compare':
        result=compare_baseline(records,symmetric=not args.fast)
    else:
        result=validate(records,d,full=args.part is None,collisions=getattr(args,'collisions',False))
        if result['valid'] and args.command=='build':
            formats=() if args.format=='none' else tuple(args.format.split(','))
            if formats:
                exported=write(records,args.output,formats,args.individual,args.name)
                result['step_roundtrip']=exported['step_roundtrip']
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/(args.command+'_report.json')).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    ok=result.get('pass',result.get('valid',False))
    print(json.dumps({'command':args.command,'pass':ok,'components':len(records),
                      'errors':result.get('failed',result.get('errors',[]))},ensure_ascii=True))
    if not ok:raise SystemExit(1)

if __name__=='__main__':main()
