#!/usr/bin/env python3
"""Sequential, auditable fixed-seed fresh-world acceptance runs (WSL)."""
import argparse,json,os,shutil,subprocess,time
from pathlib import Path
import numpy as np


def summarize(runs,requested):
    good=[r for r in runs if r['exit_code']==0 and r.get('cycle',{}).get('status')=='passed']
    summary={'requested':requested,'attempted':len(runs),'successful':len(good),'runs':runs}
    if good:
        mass=np.array([r['cycle']['loaded_volume_m3'] for r in good]);force=np.array([r['cycle']['peak_force_n'] for r in good])
        parking=np.array([[leg['final_pose'][axis] for axis in ('x','y','yaw')] for r in good for leg in r['cycle']['legs']]).reshape(len(good),-1,3)
        yaw=parking[:,:,2];yawmean=np.arctan2(np.sin(yaw).mean(axis=0),np.cos(yaw).mean(axis=0))
        summary.update(loaded_volume_cv=float(mass.std()/mass.mean()),peak_force_cv=float(force.std()/force.mean()),
          parking_position_max_dispersion_m=float(np.linalg.norm(parking[:,:,:2]-parking[:,:,:2].mean(axis=0),axis=2).max()),
          parking_yaw_max_dispersion_deg=float(np.degrees(np.abs(np.arctan2(np.sin(yaw-yawmean),np.cos(yaw-yawmean))).max())),
          minimum_rtf=min(r['cycle']['observed_rtf'] for r in good))
    summary['gates']={'all_cycles_passed':len(good)==requested,
      'loaded_volume_cv_le_1_percent':summary.get('loaded_volume_cv',float('inf'))<=.01,
      'peak_force_cv_le_2_percent':summary.get('peak_force_cv',float('inf'))<=.02,
      'parking_dispersion_le_5_cm':summary.get('parking_position_max_dispersion_m',float('inf'))<=.05,
      'parking_yaw_dispersion_le_half_degree':summary.get('parking_yaw_max_dispersion_deg',float('inf'))<=.5,
      'control_rtf_ge_0_9':summary.get('minimum_rtf',0)>=.9}
    summary['status']='passed' if all(summary['gates'].values()) else ('running' if len(runs)<requested else 'failed')
    return summary


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--runs',type=int,default=10);parser.add_argument('--seed',type=int,default=1001)
    parser.add_argument('--localization',choices=['none','lio_map'],default='lio_map');parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=Path(__file__).resolve().parents[2];args.output.mkdir(parents=True,exist_ok=False)
    frozen=args.output/'run_frozen.sh';shutil.copyfile(root/'scripts/wsl/run_loader_soil_demo.sh',frozen)
    runtime=Path('/home/lyc/loader_sim_runtime/results/runs');runs=[]
    for index in range(args.runs):
        if (args.output/'STOP').exists():
            summary=summarize(runs,args.runs);summary.update(status='aborted',reason='STOP marker requested a pause between complete runs')
            (args.output/'summary.json').write_text(json.dumps(summary,indent=2));break
        before=set(runtime.glob('*'));started=time.time()
        env=os.environ|{'LOADER_PROJECT_ROOT':str(root),'LOADER_HEADLESS':'1','LOADER_RANDOM_SEED':str(args.seed)}
        mode='perception' if args.localization!='none' else 'physics'
        with (args.output/f'run_{index+1:02d}.log').open('w') as log:
            completed=subprocess.run(['bash',str(frozen),mode,'auto',args.localization,'ab'],env=env,stdout=log,stderr=subprocess.STDOUT)
        created=sorted(set(runtime.glob('*'))-before);record={'run':index+1,'seed':args.seed,'exit_code':completed.returncode,'wall_seconds':time.time()-started}
        if len(created)==1:
            folder=created[0];record['runtime_directory']=str(folder)
            evidence=args.output/f'run_{index+1:02d}';evidence.mkdir()
            for relative in ('ab_cycle.json','localization/metrics.json','localization/status.json','runtime_profile.json','performance.json'):
                source=folder/relative
                if source.exists():
                    target=evidence/relative;target.parent.mkdir(exist_ok=True);shutil.copyfile(source,target)
            if (folder/'ab_cycle.json').exists():
                cycle=json.loads((folder/'ab_cycle.json').read_text());cycle.pop('trace',None);record['cycle']=cycle
        else:record['error']='expected exactly one fresh run directory'
        runs.append(record);summary=summarize(runs,args.runs)
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
        print(f'Acceptance {index+1}/{args.runs}: exit {completed.returncode}, successful {summary["successful"]}',flush=True)
    if summary['status']!='passed':raise SystemExit(1)


if __name__=='__main__':main()
