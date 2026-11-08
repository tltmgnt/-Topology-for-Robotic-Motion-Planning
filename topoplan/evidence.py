"""Captured benchmark receipts and independently recomputed, offline review bundles."""
import hashlib
from html import escape
import json
import math
import os
from pathlib import Path
import shutil
import stat
from .experiments import normalize, execute, summarize

LIMIT=8*1024*1024
FILES={'study.json','receipts.json','report.json','review.html','manifest.json'}


def canonical(value):
    return (json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()


def decode(raw):
    def pairs(items):
        result={}
        for k,v in items:
            if k in result:raise ValueError('duplicate JSON key')
            result[k]=v
        return result
    try:
        value=json.loads(raw.decode('utf-8-sig'),object_pairs_hook=pairs,
                         parse_constant=lambda _:(_ for _ in ()).throw(ValueError('nonfinite JSON number')))
    except (RecursionError,UnicodeError) as error:raise ValueError('invalid UTF8/JSON nesting') from error
    pending=[(value,0)]
    while pending:
        child,depth=pending.pop()
        if depth>64:raise ValueError('JSON nesting bound exceeded')
        if isinstance(child,float) and not math.isfinite(child):raise ValueError('nonfinite JSON number')
        if isinstance(child,dict):pending.extend((v,depth+1) for v in child.values())
        elif isinstance(child,list):pending.extend((v,depth+1) for v in child)
    return value


def read(path):
    if Path(path).is_symlink():raise ValueError('symlink input forbidden')
    fd=os.open(path,os.O_RDONLY|getattr(os,'O_NONBLOCK',0)|getattr(os,'O_NOFOLLOW',0))
    with os.fdopen(fd,'rb') as stream:
        before=os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or before.st_size>LIMIT:raise ValueError('bounded regular file required')
        raw=stream.read(LIMIT+1);after=os.fstat(stream.fileno())
        if len(raw)>LIMIT or (before.st_size,before.st_mtime_ns,before.st_ctime_ns)!=(after.st_size,after.st_mtime_ns,after.st_ctime_ns):
            raise ValueError('input too large or changed during reading')
    return raw


def render(report):
    rows=[]
    for exp in report['experiments']:
        lengths=', '.join(f'{r["length"]:.6g}' for r in exp.get('routes',[])) or '—'
        rows.append('<tr>'+''.join('<td>'+escape(str(v))+'</td>' for v in
                    (exp['id'],exp.get('planner') or exp['kind'],exp['radius'],exp['termination'],
                     exp.get('distinct_signatures','—'),lengths))+'</tr>')
    return ('<!doctype html><html lang="en"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            '<meta http-equiv="Content-Security-Policy" content="default-src \'none\'; style-src \'unsafe-inline\'">'
            '<title>TopoPlan benchmark evidence</title><style>body{font:16px system-ui;max-width:1100px;margin:24px auto;padding:0 16px}'
            'table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccd;overflow-wrap:anywhere}pre{white-space:pre-wrap;overflow-wrap:anywhere}.table{overflow:auto}</style></head><body>'
            '<h1>Topological diversity and bounded planning</h1><p>Offline synthetic experiments. '
            'Signatures are homology-style summaries, not complete homotopy classes. Timing is a captured observation. '
            'No robot execution, perception model or safety certification.</p><div class="table"><table><thead><tr>'
            '<th>Experiment</th><th>Planner</th><th>Radius</th><th>Termination</th><th>Signatures</th><th>Route lengths</th>'
            '</tr></thead><tbody>'+''.join(rows)+'</tbody></table></div><h2>Independent checks and complete evidence</h2><pre>'
            +escape(json.dumps(report,sort_keys=True,indent=2))+'</pre></body></html>\n')


def contents(study,receipts):
    study=normalize(study)
    report=summarize(study,receipts)
    sources=['core.py','geometry.py','continuous.py','temporal.py','audit.py','experiments.py',
             'evidence.py','baselines.py','ompl_worker.py','benchmark.py']
    report['implementation_sha256']={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest() for name in sources}
    report['study_sha256']=hashlib.sha256(canonical(study)).hexdigest()
    report['receipts_sha256']=hashlib.sha256(canonical(receipts)).hexdigest()
    files={'study.json':canonical(study),'receipts.json':canonical(receipts),'report.json':canonical(report),
           'review.html':render(report).encode()}
    files['manifest.json']=canonical({'schema_version':1,'engine_version':'0.2.0',
        'files':{name:{'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw)} for name,raw in sorted(files.items())}})
    if any(len(raw)>LIMIT for raw in files.values()):raise ValueError('bundle output exceeds 8 MiB per file')
    return files,report


def create_bundle(study,destination):
    study=normalize(study);path=Path(destination);path.mkdir(mode=0o700)
    try:
        receipts=[execute(exp) for exp in study['experiments']]
        files,report=contents(study,receipts)
        for name,raw in files.items():
            with (path/name).open('xb') as stream:stream.write(raw)
    except BaseException:
        shutil.rmtree(path)
        raise
    return report


def verify_bundle(destination):
    path=Path(destination)
    if path.is_symlink() or not path.is_dir() or {p.name for p in path.iterdir()}!=FILES:
        raise ValueError('exact nonsymlink benchmark bundle required')
    actual={name:read(path/name) for name in sorted(FILES)}
    expected,report=contents(decode(actual['study.json']),decode(actual['receipts.json']))
    if actual!=expected:raise ValueError('benchmark evidence differs from independent recomputation')
    return report
