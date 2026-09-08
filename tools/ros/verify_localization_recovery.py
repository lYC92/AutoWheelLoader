#!/usr/bin/env python3
"""Score localization outages against independent vehicle/truth observations."""
import argparse,json,math
from pathlib import Path

def verify(directory):
    events=json.loads((directory/'health_events.json').read_text());tracking=False;active=None;episodes=[]
    for event in events:
        if event['status']=='tracking':
            if active is not None:active['recovered_at_s']=event['sim_time'];episodes.append(active);active=None
            tracking=True
        elif tracking:
            if active is None:active={'started_at_s':event['sim_time'],'initial_speed_mps':event['vehicle_speed_mps'],'samples':[]}
            active['samples'].append(event)
    if active is not None:episodes.append(active)
    results=[]
    for episode in episodes:
        samples=episode.pop('samples');origin=samples[0]['truth_position'];speed=abs(episode['initial_speed_mps'])
        distance=max(math.dist(origin,s['truth_position']) for s in samples if origin and s['truth_position'])
        bound=speed*speed/(2*.5)+speed*.3+.3
        delayed=[abs(s['vehicle_speed_mps']) for s in samples if s['sim_time']-episode['started_at_s']>=.5]
        episode.update(maximum_truth_displacement_m=distance,stopping_corridor_length_m=bound,
          maximum_speed_after_half_second_mps=max(delayed,default=0.),
          passed='recovered_at_s' in episode and distance<=bound and (not delayed or max(delayed)<.05))
        results.append(episode)
    passed=any(abs(e['initial_speed_mps'])>.2 for e in results) and all(e['passed'] for e in results)
    report={'status':'passed' if passed else 'failed','episodes':results,'pose_source':'independent truth for scoring only'}
    (directory/'recovery_verification.json').write_text(json.dumps(report,indent=2))
    if not passed:raise RuntimeError('moving localization stop/recovery was not demonstrated')
    print(f'PASS localization loss braking and recovery: {len(results)} episodes')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);verify(parser.parse_args().directory)
