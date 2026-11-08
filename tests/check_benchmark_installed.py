"""Installed, isolated CLI and benchmark bundle workflows outside the checkout."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    root=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)
        def run(module,args,expected=0):
            result=subprocess.run([sys.executable,'-I','-m',module,*map(str,args)],cwd=path,capture_output=True,text=True)
            if result.returncode!=expected:raise AssertionError(result.stderr or result.stdout)
            return json.loads(result.stdout or result.stderr)
        legacy=run('topoplan',[root/'examples/obstacle.json','--routes','2'])
        assert len(legacy['routes'])==2
        run('topoplan.benchmark',['generate','--output',path/'study.json'])
        run('topoplan.benchmark',['run',path/'study.json','--output',path/'review'])
        report=run('topoplan.benchmark',['verify',path/'review'])
        assert report['bundle_consistent'] and report['experiments']==20
        # Corruption must fail even though the study remains valid.
        (path/'review/report.json').write_text('{}')
        run('topoplan.benchmark',['verify',path/'review'],2)
    print('Installed legacy CLI, 20-experiment run, offline verification and tamper rejection passed')


if __name__=='__main__':main()
