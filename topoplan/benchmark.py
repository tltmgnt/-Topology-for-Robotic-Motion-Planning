"""Create standardized synthetic studies, run them, and verify offline evidence."""
import argparse
import json
import sys
from .evidence import canonical,decode,read,create_bundle,verify_bundle
from .experiments import normalize


def standard_study(include_ompl=False):
    cases=[('island',['.........','.........','....#....','....#....','....#....','.........','.........'],[1,3],[7,3]),
           ('narrow',['#########','.........','#########'],[1,1],[7,1]),
           ('disconnected',['....#....']*5,[1,2],[7,2])]
    planners=['astar','rrt']+(['ompl:PRM','ompl:RRT','ompl:RRTstar'] if include_ompl else [])
    experiments=[]
    for name,rows,start,goal in cases:
        for radius in (0,.3,.55):
            for planner in planners:
                experiments.append({'id':f'{name}-{radius}-{planner}','kind':'static','grid':rows,'start':start,'goal':goal,
                                    'radius':radius,'planner':planner,'routes':2,'seed':7,'iterations':500,
                                    'max_expansions':10000,'seconds':.15})
    experiments += [
        {'id':'moving-crossing','kind':'dynamic','grid':['.....']*5,'start':[0,2],'goal':[4,2],
         'radius':.2,'horizon':10,'obstacles':[{'radius':.2,'path':[[2,0],[2,1],[2,2],[2,3],[2,4]]}]},
        {'id':'robot-swap','kind':'multi','grid':['.....']*3,'radius':.2,'horizon':12,
         'robots':[{'id':'a','start':[0,1],'goal':[4,1]},{'id':'b','start':[4,1],'goal':[0,1]}]}]
    return {'schema_version':1,'experiments':experiments}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    generate=commands.add_parser('generate');generate.add_argument('--output',required=True);generate.add_argument('--include-ompl',action='store_true')
    run=commands.add_parser('run');run.add_argument('study');run.add_argument('--output',required=True)
    verify=commands.add_parser('verify');verify.add_argument('bundle')
    args=parser.parse_args(argv)
    try:
        if args.command=='generate':
            study=standard_study(args.include_ompl);normalize(study)
            with open(args.output,'xb') as stream:stream.write(canonical(study))
            result={'study_created':True,'experiments':len(study['experiments'])}
        elif args.command=='run':
            report=create_bundle(decode(read(args.study)),args.output)
            result={'bundle_created':True,'experiments':len(report['experiments']),'execution_authorized':False}
        else:
            report=verify_bundle(args.bundle)
            result={'bundle_consistent':True,'experiments':len(report['experiments']),
                    'execution_authorized':False,'source_authenticated':False}
    except (OSError,ValueError,TypeError,KeyError,OverflowError) as error:
        print(json.dumps({'error':str(error),'execution_authorized':False}),file=sys.stderr);return 2
    print(json.dumps(result,sort_keys=True));return 0


if __name__=='__main__':raise SystemExit(main())
