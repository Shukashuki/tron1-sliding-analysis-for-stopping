"""Publish reports and plots from a completed fixed parameter-validation suite."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    cases=json.loads((ROOT/'config/validation_suite.json').read_text())
    base=json.loads((ROOT/'config/stopping.json').read_text())
    reports=[]
    for i,case in enumerate(cases,1):
        path=args.run/f'trial-{i:03d}'
        r=json.loads((path/'report.json').read_text())
        assert r['case_name']==case['name']
        assert r['settings']==base | case['settings']
        for field,name in [('script_sha256','stopping_lab.py'),('core_sha256','stopping_core.py')]:
            assert r[field]==hashlib.sha256((ROOT/'scripts'/name).read_bytes()).hexdigest()
        assert abs(r['effective_physics_dt']-.005)<1e-9
        reports.append((path,r))
    args.output.mkdir(parents=True,exist_ok=False)
    summary={'engine':'Isaac Sim 4.5 / PhysX, Windows, RTX 3060 Ti',
             'scope':'one deterministic run per setting plus one repeated nominal run; controller development validation',
             'cases':[]}
    for path,r in reports:
        target=args.output/r['case_name']
        target.mkdir()
        shutil.copy2(path/'report.json',target/'report.json')
        shutil.copy2(path/'settings.json',target/'settings.json')
        subprocess.run([sys.executable,str(ROOT/'scripts/plot_trial.py'),str(path),'--output',str(target/'overview.png')],check=True)
        summary['cases'].append({k:r[k] for k in ('case_name','settings','termination','stable_stop',
            'signed_stop_distance_m','stop_time_after_brake_s','max_excursion_after_brake_m','final_displacement_after_brake_m',
            'max_supported_slip_after_brake_mps','max_abs_pitch_deg','samples')})
    nominal=next(path for path,r in reports if r['case_name']=='nominal')
    repeated=next(path for path,r in reports if r['case_name']=='nominal_repeat')
    summary['nominal_repeat_csv_identical']=((nominal/'telemetry.csv').read_bytes()==(repeated/'telemetry.csv').read_bytes())
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
