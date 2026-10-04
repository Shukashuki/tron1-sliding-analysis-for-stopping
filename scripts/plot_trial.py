"""Plot one saved trial's actual motion, wheel slip, torque and support."""
import argparse
import csv
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("trial",type=Path)
    p.add_argument("--output",type=Path)
    args=p.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    with (args.trial/"telemetry.csv").open() as stream:
        rows=list(csv.DictReader(stream))
    data={key:np.array([float(r[key]) for r in rows]) for key in rows[0]}
    report=json.loads((args.trial/"report.json").read_text())
    t=data['t']
    fig,ax=plt.subplots(3,2,figsize=(12,9),sharex=True,layout='constrained')
    for key,label in [('vx','Axle'),('reference_v','Reference'),('rolling_l','Left wheel surface'),('rolling_r','Right wheel surface')]:
        ax[0,0].plot(t,data[key],label=label)
    ax[0,0].set_ylabel('Speed (m/s)')
    ax[0,1].plot(t,data['x'],label='Actual axle x')
    ax[0,1].plot(t,data['reference_x'],'--',label='Reference x')
    ax[0,1].set_ylabel('Position (m)')
    for side in ['l','r']:
        ax[1,0].plot(t,data['slip_'+side],label=side.upper())
        ax[2,0].plot(t,data['torque_'+side],label=side.upper())
        ax[2,1].plot(t,data['normal_'+side],label=side.upper())
    ax[1,0].set_ylabel('Slip speed (m/s)')
    ax[1,1].plot(t,np.degrees(data['pitch']),label='COM pitch')
    ax[1,1].plot(t,np.degrees(data['roll']),label='Base roll')
    ax[1,1].set_ylabel('Angle (deg)')
    ax[2,0].set_ylabel('Applied wheel torque (Nm)')
    ax[2,1].set_ylabel('Net contact Z proxy (N)')
    for a in ax.flat:
        a.grid(alpha=.2)
        a.legend(fontsize=8)
        braking=np.flatnonzero(data['braking'])
        if len(braking):a.axvline(t[braking[0]],color='k',ls=':',alpha=.5)
    for a in ax[-1]:a.set_xlabel('Simulation time (s)')
    cfg=report['settings']
    fig.suptitle(f"TRON1 stopping | v0={cfg['initial_speed']} m/s | mu={cfg['static_friction']}/{cfg['dynamic_friction']} | stable stop={report['stable_stop']}")
    target=args.output or args.trial/'overview.png'
    target.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(target,dpi=150)
    print(target)


if __name__=='__main__':main()
