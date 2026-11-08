"""Plan paths from a JSON grid and export a JSON report and optional SVG."""
import argparse
import json
from pathlib import Path
from . import Grid


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--routes',type=int,default=1)
    parser.add_argument('--max-expansions',type=int,default=100000)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--svg',type=Path)
    args=parser.parse_args(argv)
    try:
        data=json.loads(args.input.read_text(encoding='utf-8'))
        if not isinstance(data,dict): raise ValueError('Input must be a JSON object')
        grid=Grid(data['grid'],data.get('radius',0))
        report=grid.plan(data['start'],data['goal'],args.routes,args.max_expansions)
        encoded=json.dumps(report,indent=2,allow_nan=False)+'\n'
        for path,content in [(args.output,encoded),(args.svg,grid.svg(report))]:
            if path:
                path.parent.mkdir(parents=True,exist_ok=True); path.write_text(content,encoding='utf-8')
        print(encoded,end='')
        return 0 if report['routes'] else 2
    except (OSError,ValueError,KeyError,TypeError,OverflowError) as exc:
        parser.error(str(exc))


if __name__=='__main__': raise SystemExit(main())
