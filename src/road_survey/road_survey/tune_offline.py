"""Re-classify a saved survey with different thresholds -- no simulator needed.

Fly the survey once, save the ``.npz``, then iterate on the thresholds here in
a second per run instead of a five-minute flight per run.

    ros2 run road_survey tune_offline ~/uav_guided_ugv/maps/road_map.npz \
        --slope 12 --step 0.3 --rough 0.15 --thresh 0.35 -o /tmp/try1

    ros2 run road_survey tune_offline road_map.npz --sweep slope 8 10 12 15 20
"""

from __future__ import annotations

import argparse
import os

import numpy as np

from . import costmap as cm
from . import mapio
from . import risk as rk


def build_args():
    ap = argparse.ArgumentParser(prog='tune_offline', description=__doc__)
    ap.add_argument('npz')
    ap.add_argument('--slope', type=float, default=None, help='slope_crit_deg')
    ap.add_argument('--step', type=float, default=None, help='step_crit_m')
    ap.add_argument('--rough', type=float, default=None, help='rough_crit_m')
    ap.add_argument('--thresh', type=float, default=None, help='risk_thresh')
    ap.add_argument('--window', type=int, default=None, help='window_cells')
    ap.add_argument('--min-count', type=int, default=None)
    ap.add_argument('--min-area', type=float, default=None, help='min_area_m2')
    ap.add_argument('--edge-ref', type=float, default=None, help='edge_cost_ref_m')
    ap.add_argument('--centerline', action='store_true')
    ap.add_argument('--sweep', nargs='+', default=None,
                    metavar=('FIELD', 'VALUE'),
                    help='one of slope/step/rough/thresh followed by values')
    ap.add_argument('-o', '--out', default=None, help='output prefix')
    return ap


def _params(args, data):
    p = rk.RiskParams()
    if args.slope is not None:
        p.slope_crit_deg = args.slope
    if args.step is not None:
        p.step_crit_m = args.step
    if args.rough is not None:
        p.rough_crit_m = args.rough
    if args.thresh is not None:
        p.risk_thresh = args.thresh
    if args.window is not None:
        p.window_cells = args.window
    if args.min_count is not None:
        p.min_count = args.min_count
    c = cm.CostmapParams()
    if args.min_area is not None:
        c.min_area_m2 = args.min_area
    if args.edge_ref is not None:
        c.edge_cost_ref_m = args.edge_ref
    del data
    return p, c


def run_once(data, p: rk.RiskParams, c: cm.CostmapParams, out_prefix: str,
             centerline: bool = False):
    elev = data['elevation']
    known = data['count'] >= p.min_count
    res = data['resolution']

    risk, road, valid, feat = rk.classify(elev, known, res, p)
    road, info = cm.clean_mask(road, res, c)
    occ = cm.to_occupancy(road, valid, res, c)

    cl_rc = None
    if centerline:
        from .centerline import longest_skeleton_path  # noqa: PLC0415
        cl_rc = longest_skeleton_path(road)

    png = mapio.save_debug_png(out_prefix + '.png', elevation=elev, road=road,
                               risk=risk, resolution=res, centerline_rc=cl_rc)
    mapio.save_npz(out_prefix + '.npz', occ=occ, elevation=elev, risk=risk,
                   road=road, valid=valid, count=data['count'],
                   extent=data['extent'], origin_x=data['origin_x'],
                   origin_y=data['origin_y'], resolution=res,
                   frame_id=data['frame_id'],
                   params={'risk': p.as_dict(), 'costmap': c.as_dict()})
    # 2 * distance-to-edge peaks at the true width along the road's spine, so
    # the high percentile is the width estimate; the median is meaningless.
    width = cm.road_width_m(road, res)
    on = width[road] if road.any() else np.zeros(1)
    info.update({
        'median_slope_on_road_deg': float(np.median(feat.slope_deg[road]))
        if road.any() else float('nan'),
        'road_width_p95_m': float(np.percentile(on, 95)),
        'road_width_max_m': float(on.max()),
        'png': png,
    })
    return info


def main(argv=None):
    args = build_args().parse_args(argv)
    data = mapio.load_npz(os.path.expanduser(args.npz))
    out = args.out or os.path.splitext(os.path.expanduser(args.npz))[0] + '_tuned'

    if args.sweep:
        field, values = args.sweep[0], [float(v) for v in args.sweep[1:]]
        attr = {'slope': 'slope_crit_deg', 'step': 'step_crit_m',
                'rough': 'rough_crit_m', 'thresh': 'risk_thresh'}[field]
        for v in values:
            p, c = _params(args, data)
            setattr(p, attr, v)
            info = run_once(data, p, c, f'{out}_{field}{v:g}')
            print(f'{field}={v:g}  area={info["road_area_m2"]:8.1f} m2  '
                  f'width_p95={info["road_width_p95_m"]:5.2f} m  '
                  f'parts={info["components_found"]}  -> {info["png"]}')
        return 0

    p, c = _params(args, data)
    info = run_once(data, p, c, out, args.centerline)
    for k, v in info.items():
        print(f'{k}: {v}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
