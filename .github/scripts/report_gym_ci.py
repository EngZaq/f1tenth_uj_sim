"""Post-run simulation evidence; never connected to motion commands."""
import csv,json,math,subprocess,sys
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from sim_geometry import BODY_FRONT,BODY_REAR,BODY_HALFWIDTH

def read_csv(path):
    with path.open() as stream:return list(csv.DictReader(stream))
def finite(rows,key):
    values=np.array([float(r.get(key,'nan')) for r in rows])
    return values[np.isfinite(values)]
def distribution(values):
    if not len(values):return None
    return dict(samples=len(values),rms=float(np.sqrt(np.mean(values**2))),
                p95=float(np.percentile(values,95)),maximum=float(values.max()))

statuses=list((ROOT/'results/missions').glob('autonomous_*/mission_status.json'))
if not statuses:
    print('No automatic mission output exists; inspect dependency/preflight failure.')
    sys.exit(0)
assert len(statuses)==1,'CI must contain exactly one fresh automatic mission'
folder=statuses[0].parent
status=json.loads(statuses[0].read_text())
files=list(folder.glob('race_*/estimated_drive.csv'))
if not files:
    print(json.dumps(status,indent=2))
    raise SystemExit('Mission stopped before controller evidence was produced')
drivefile=files[0];race=drivefile.parent;drive=read_csv(drivefile)
moving=[r for r in drive if r['state']=='DRIVE']
result=dict(stage=status['stage'],stages=[e['stage'] for e in status['events']],
    elapsed_s=status['events'][-1]['time_s']-status['events'][0]['time_s'],
    max_estimated_laps=max(int(r['lap']) for r in drive),controller_samples=len(drive),
    driving_samples=len(moving),last_controller=drive[-1],
    estimated_cte_m=distribution(finite(moving,'cte_m')),
    minimum_predicted_gap_m=float(finite(moving,'predicted_body_gap_m').min()) if moving else None,
    local_speed_limited_samples=sum(r.get('local_speed_limited')=='True' for r in moving),
    watchdog_fault=(folder/'watchdog_fault.json').exists())
truthfile=race/'localization_truth_audit.csv'
if truthfile.exists():
    truth=read_csv(truthfile)
    if len(truth)>1:
        xy=np.array([[float(r[k]) for k in ('truth_x_sim','truth_y_sim')] for r in truth])
        heading=np.radians([float(r['truth_yaw_deg']) for r in truth])
        times=np.array([float(r['truth_stamp_s']) for r in truth])
        result.update(truth_samples=len(truth),truth_reference_frames=sorted(set(r.get('truth_reference_frame','legacy') for r in truth)),
            truth_distance_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
            truth_turns=float((np.unwrap(heading)[-1]-np.unwrap(heading)[0])/(2*math.pi)),
            truth_endpoint_distance_m=float(np.linalg.norm(xy[-1]-xy[0])),
            truth_span_s=float(times[-1]-times[0]),truth_max_sample_gap_s=float(np.diff(times).max()))
        if result['truth_reference_frames']==['rear_axle']:
            mapfile=ROOT/'maps/uj_technical_loop_v1.yaml'
            meta=yaml.safe_load(mapfile.read_text());image=mapfile.parent/meta['image']
            pixels=np.asarray(Image.open(image).convert('L'));res=float(meta['resolution'])
            ox,oy,angle=meta['origin']
            lon,lat=np.meshgrid(np.linspace(BODY_REAR,BODY_FRONT,30),np.linspace(-BODY_HALFWIDTH,BODY_HALFWIDTH,17))
            lon=lon.ravel();lat=lat.ravel();c=np.cos(heading)[:,None];s=np.sin(heading)[:,None]
            bx=xy[:,0,None]+c*lon-s*lat;by=xy[:,1,None]+s*lon+c*lat
            mx=math.cos(angle)*(bx-ox)+math.sin(angle)*(by-oy)
            my=-math.sin(angle)*(bx-ox)+math.cos(angle)*(by-oy)
            col=np.floor(mx/res).astype(int);row=pixels.shape[0]-1-np.floor(my/res).astype(int)
            valid=(row>=0)&(row<pixels.shape[0])&(col>=0)&(col<pixels.shape[1])
            distance=distance_transform_edt(pixels>=250)*res
            clearance=np.where(valid,distance[np.clip(row,0,pixels.shape[0]-1),np.clip(col,0,pixels.shape[1]-1)],0.)
            result['sampled_true_body_minimum_clearance_m']=float(clearance.min())
            result['occupied_or_outside_true_body_samples']=int((clearance<=0).sum())
            result['clearance_caveat']='Sampled poses/body grid against simulator raster; not a continuous-time or hardware certificate.'
resultfile=race/'race_result.json'
if resultfile.exists():result['race_result']=json.loads(resultfile.read_text())
print('MISSION_EVIDENCE_JSON_BEGIN',flush=True)
print(json.dumps(result,indent=2),flush=True)
print('MISSION_EVIDENCE_JSON_END',flush=True)
(folder/'ci_acceptance_evidence.json').write_text(json.dumps(result,indent=2)+'\n')
if truthfile.exists():
    subprocess.run([sys.executable,str(ROOT/'tools/localization/summarize_sim_localization_audit.py'),
        str(truthfile),'--controller-csv',str(drivefile)],check=True)
guard=race/'guard_stop_context.json'
if guard.exists() and status['stage']!='COMPLETED_ESTIMATED_LAPS':
    print('GUARD_JSON_BEGIN',flush=True);print(guard.read_text(),flush=True);print('GUARD_JSON_END',flush=True)
assert status['stage']=='COMPLETED_ESTIMATED_LAPS','Full mission did not complete'
requested=result['race_result']['requested_laps']
assert result['max_estimated_laps']==requested
assert drive[-1]['state']=='STOP' and float(drive[-1]['cmd_speed_mps'])==0.
assert not result['watchdog_fault']
assert result.get('truth_reference_frames')==['rear_axle']
route=np.loadtxt(folder/'new_track_body_optimized.csv',delimiter=';',comments='#')
length=float(route[-1,0]+np.linalg.norm(route[-1,1:3]-route[0,1:3]))
assert result['truth_distance_m']>=.85*requested*length,'Insufficient independent physical travel'
assert abs(abs(result['truth_turns'])-requested)<.2,'Physical heading winding disagrees with requested laps'
assert result['truth_endpoint_distance_m']<1.,'Physical route did not return near its start'
assert result['occupied_or_outside_true_body_samples']==0,'Sampled true body intersects the simulator raster'
print('PASS: requested laps, stopped command, independent physical travel and sampled body occupancy checks')
