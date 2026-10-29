import csv,json,math,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()/'tools/navigation'))
import trajectory_safety as safety
folders=list(Path('downloaded').rglob('mission_status.json'))
assert len(folders)==1,folders
folder=folders[0].parent
status=json.loads(folders[0].read_text())
drivefile=next(folder.glob('race_*/estimated_drive.csv'))
race=drivefile.parent
with drivefile.open() as f:rows=list(csv.DictReader(f))
finite=[r for r in rows if math.isfinite(float(r['x']))]
print('STATUS_JSON_BEGIN')
print(json.dumps(dict(stage=status['stage'],last_finite_controller=finite[-1] if finite else None,
 max_laps=max(int(r['lap']) for r in rows),watchdog_fault=(folder/'watchdog_fault.json').exists()),indent=2))
print('STATUS_JSON_END')
guard=race/'guard_stop_context.json'
if guard.exists():
 d=json.loads(guard.read_text())
 print('GUARD_JSON_BEGIN');print(json.dumps(d));print('GUARD_JSON_END')
 original=safety.stopping_horizon
 result=[]
 for buffer in (.10,.02,0.):
  safety.stopping_horizon=lambda v,reaction=.35,deceleration=.7,buffer=buffer:max(0.,v)*reaction+max(0.,v)**2/(2*deceleration)+buffer
  for proposal in d['proposals']:
   blocked,gap,hits=safety.command_swept_collision(
    d['scan_ranges'],d['angle_min'],d['angle_increment'],d['range_min'],d['range_max'],
    proposal['steering'],proposal['guard_velocity'],initial_steering=d['initial_steering'],
    scan_to_body=d['scan_to_body'],reaction=d['reaction'],margin=d['margin'])
   result.append(dict(buffer_m=buffer,speed_mps=proposal['speed'],gap_m=gap,blocked=blocked))
 safety.stopping_horizon=original
 print('BUFFER_REPLAY_JSON_BEGIN');print(json.dumps(result,indent=2));print('BUFFER_REPLAY_JSON_END')
 route=np.loadtxt(folder/'new_track_body_optimized.csv',delimiter=';',comments='#')
 i=int(finite[-1]['index'])
 print('ROUTE_WINDOW_JSON_BEGIN');print(json.dumps(dict(original_index=i,
 start=max(0,i-20),rows=route[max(0,i-20):i+30].tolist())));print('ROUTE_WINDOW_JSON_END')
truth=race/'localization_truth_audit.csv'
if truth.exists():
 import subprocess
 subprocess.run([sys.executable,'tools/localization/summarize_sim_localization_audit.py',
 str(truth),'--controller-csv',str(drivefile)],check=True)
