"""Mask cleanup and occupancy encoding."""

import numpy as np

from road_survey.costmap import (CostmapParams, clean_mask, distance_to_edge,
                                 occupancy_to_pgm, road_width_m, to_occupancy)

RES = 0.25


def strip(width_m=6.0, n=120):
    m = np.zeros((n, n), dtype=bool)
    half = int(round(width_m / 2 / RES))
    m[n // 2 - half:n // 2 + half, :] = True
    return m


def test_distance_transform_is_in_metres():
    m = strip(6.0)
    d = distance_to_edge(m, RES)
    assert np.isclose(d.max(), 3.0, atol=RES)
    assert np.isclose(road_width_m(m, RES).max(), 6.0, atol=2 * RES)
    assert d[0, 0] == 0.0


def test_cost_falls_to_zero_at_the_road_centre():
    m = strip(6.0)
    occ = to_occupancy(m, np.ones_like(m), RES, CostmapParams(edge_cost_ref_m=1.5))
    n = m.shape[0]
    assert occ[n // 2, n // 2] == 0            # centre is free
    assert occ[n // 2 - 12, n // 2] > 75       # the outermost road cell is dear
    assert occ[0, 0] == 100                    # off-road is lethal
    assert occ.dtype == np.int8


def test_unknown_can_stay_unknown():
    m = strip(6.0)
    valid = np.zeros_like(m)
    valid[40:80, :] = True
    occ = to_occupancy(m, valid, RES, CostmapParams(unknown_is_lethal=False))
    assert occ[0, 0] == -1
    assert occ[m].min() >= 0


def test_small_blobs_are_dropped_and_the_road_survives():
    m = strip(6.0)
    m[5:8, 5:8] = True  # 0.75 x 0.75 m speck, ~0.6 m2
    cleaned, info = clean_mask(m, RES, CostmapParams(min_area_m2=25.0))
    assert not cleaned[6, 6]
    assert cleaned[m.shape[0] // 2, 60]
    assert info['components_kept'] == 1
    assert info['road_area_m2'] > 100.0


def test_seed_keeps_a_smaller_component():
    m = np.zeros((120, 120), dtype=bool)
    m[10:100, 10:40] = True   # big blob
    m[10:60, 80:110] = True   # smaller blob, contains the seed
    seed = (95, 30)           # (ix, iy)
    cleaned, info = clean_mask(m, RES, CostmapParams(keep_components=1), seed)
    assert cleaned[30, 95]
    assert cleaned[50, 20]
    assert info['components_kept'] == 2
    assert info['seed_in_road']


def test_opening_removes_single_cell_speckle():
    m = strip(6.0)
    m[100, 100] = True
    cleaned, _ = clean_mask(m, RES, CostmapParams())
    assert not cleaned[100, 100]


def test_pgm_flips_rows_and_marks_unknown():
    occ = np.array([[0, 100], [-1, 50]], dtype=np.int8)
    pgm = occupancy_to_pgm(occ)
    assert pgm.shape == (2, 2)
    assert pgm[1, 0] == 255       # free, was row 0 before the flip
    assert pgm[1, 1] == 0         # lethal
    assert pgm[0, 0] == 205       # unknown
    assert pgm[0, 1] == 128       # mid cost


def test_empty_mask_does_not_crash():
    m = np.zeros((40, 40), dtype=bool)
    cleaned, info = clean_mask(m, RES, CostmapParams())
    assert not cleaned.any()
    assert info['road_area_m2'] == 0.0
    assert (to_occupancy(cleaned, np.ones_like(m), RES,
                         CostmapParams()) == 100).all()
