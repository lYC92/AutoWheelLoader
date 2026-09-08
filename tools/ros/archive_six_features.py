"""Copy compact evidence out of WSL runtime; never remove original records."""
from pathlib import Path
import shutil
base=Path.home()/'loader_sim_runtime/results/runs'
out=Path('docs/baselines/2026-09-08/six_features');(out/'localization_trials').mkdir(exist_ok=True)
for name in ['perception_ab_20260908_121237_504','perception_ab_20260908_121833_733','perception_ab_20260908_122205_667','perception_ab_20260908_122517_791','perception_ab_20260908_122857_532']:
    dst=out/'localization_trials'/name;dst.mkdir(exist_ok=True)
    for rel in ['ab_cycle.json','localization/metrics.json','localization/status.json']:
        src=base/name/rel
        if src.exists():shutil.copy2(src,dst/Path(rel).name)
cam=base/'physics_ab_20260908_121647_1154'/'surround';dst=out/'surround_initial';dst.mkdir(exist_ok=True)
for rel in ['manifest.json','bev/report.json','bev/frame_0000_surround.png','bev/frame_0000_bev.png','bev/frame_0000_drivable.png']:shutil.copy2(cam/rel,dst/Path(rel).name)
shutil.copy2(Path.home()/'loader_sim_runtime/results/sloped_vehicle.json',out/'sloped_vehicle.json')
passed=base/'perception_ab_20260908_125321_713';destination=out/'estimated_ab_first_pass';destination.mkdir(exist_ok=True)
for rel in ['ab_cycle.json','localization/metrics.json','localization/status.json','reference_map.npz']:
    shutil.copy2(passed/rel,destination/Path(rel).name)
(destination/'README.md').write_text("# First complete estimated-pose A/B cycle\n\nActual frontend: static map point-to-plane ICP + 15-state IMU fusion (`lio_map`).\nThe original metrics reporter mistakenly labels all world-frame fusion as KISS-ICP; the numeric results are preserved unchanged. The launcher/reporter now records the selected algorithm explicitly.\n\nLoaded/unloaded 0.700336 m3, 100% at B, full return. Independent world-frame RMSE 0.084900 m; maximum 0.390517 m. No truth alignment. Later initializer, feedback and recovery changes require their own regression.\n",encoding='utf-8')
print('Archived localization trials, sloped vehicle, first complete estimated cycle and initial camera evidence')
