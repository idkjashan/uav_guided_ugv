"""Road classification on synthetic terrain.

The canonical case: a flat shelf cut into a hillside -- exactly the geometry of
the mountain road in the DRDO worlds -- must come out as road, and the cut face
and the drop must not.
"""

import numpy as np

from road_survey.risk import (RiskParams, classify, multi_level_cells,
                              risk_from_features, terrain_features)

RES = 0.25


def hillside_with_shelf(n=200, road_half_m=3.0, slope_deg=35.0):
    """(elev, known, y-in-metres). Road runs along +x, centred in y."""
    y = (np.arange(n) + 0.5) * RES
    centre = y[n // 2]
    d = y - centre
    grad = np.tan(np.radians(slope_deg))
    prof = np.where(d > road_half_m, (d - road_half_m) * grad,
                    np.where(d < -road_half_m, (d + road_half_m) * grad, 0.0))
    elev = np.tile(prof[:, None], (1, n)).astype(np.float32)
    return elev, np.ones((n, n), dtype=bool), d


def test_flat_ground_is_all_road():
    elev = np.zeros((60, 60), dtype=np.float32)
    _, road, valid, _ = classify(elev, np.ones_like(elev, bool), RES, RiskParams())
    assert road[valid].all()


def test_shelf_is_road_and_the_slopes_are_not():
    elev, known, d = hillside_with_shelf()
    _, road, valid, feat = classify(elev, known, RES, RiskParams())

    on_road = np.abs(d) < 2.0
    off_road = np.abs(d) > 4.0
    interior = np.zeros_like(road)
    interior[8:-8, 8:-8] = True

    assert road[on_road][:, 8:-8].all()
    assert not road[off_road][:, 8:-8].any()
    assert valid[interior].all()
    # The shelf really is flat and the cut face really is 35 degrees.
    assert feat.slope_deg[on_road][:, 20:180].max() < 1.0
    assert feat.slope_deg[off_road][:, 20:180].mean() > 30.0


def test_gentle_slope_stays_road_but_a_steep_one_does_not():
    for deg, expect_road in ((8.0, True), (25.0, False)):
        grad = np.tan(np.radians(deg))
        y = (np.arange(80) + 0.5) * RES
        elev = np.tile((y * grad)[:, None], (1, 80)).astype(np.float32)
        _, road, valid, _ = classify(elev, np.ones((80, 80), bool), RES,
                                     RiskParams())
        assert road[valid].any() == expect_road


def test_a_kerb_trips_the_step_term():
    elev = np.zeros((60, 60), dtype=np.float32)
    elev[:, 30:] = 0.6  # a 0.6 m wall, well over step_crit_m
    feat = terrain_features(elev, np.ones((60, 60), bool), RES, RiskParams())
    assert feat.step_m[:, 28:32].max() > 0.25
    risk = risk_from_features(feat, RiskParams())
    assert np.isclose(risk[30, 30], 1.0)
    assert np.isclose(risk[30, 5], 0.0)


def test_roughness_is_measured_about_the_plane_not_the_mean():
    """A perfectly smooth 20-degree ramp must read as zero roughness.

    If the plane fit were replaced by a local mean, the slope would leak into
    the roughness term and every hillside would look rubble-strewn.
    """
    grad = np.tan(np.radians(20.0))
    y = (np.arange(60) + 0.5) * RES
    elev = np.tile((y * grad)[:, None], (1, 60)).astype(np.float32)
    feat = terrain_features(elev, np.ones((60, 60), bool), RES, RiskParams())
    assert feat.rough_m[10:-10, 10:-10].max() < 0.01
    assert abs(feat.slope_deg[10:-10, 10:-10].mean() - 20.0) < 0.5


def test_noise_shows_up_as_roughness():
    rng = np.random.default_rng(0)
    elev = rng.normal(0.0, 0.20, size=(80, 80)).astype(np.float32)
    p = RiskParams(smooth_cells=0)
    feat = terrain_features(elev, np.ones((80, 80), bool), RES, p)
    assert feat.rough_m[10:-10, 10:-10].mean() > 0.12
    _, road, valid, _ = classify(elev, np.ones((80, 80), bool), RES, p)
    assert not road[valid].any()


def test_unknown_cells_never_become_road():
    elev = np.full((60, 60), np.nan, dtype=np.float32)
    known = np.zeros((60, 60), dtype=bool)
    known[20:40, 20:40] = True
    elev[known] = 0.0
    _, road, valid, _ = classify(elev, known, RES, RiskParams())
    assert not road[~known].any()
    assert not valid[~known].any()


def test_multi_level_flags_stacked_road():
    road = np.zeros((10, 10), dtype=bool)
    road[2:8, 2:8] = True
    extent = np.zeros((10, 10), dtype=np.float32)
    extent[3, 3] = 5.0   # two legs of a switchback in one cell
    extent[9, 9] = 9.0   # not road, must not be flagged
    flagged = multi_level_cells(extent, road, gap_m=2.0)
    assert flagged[3, 3]
    assert flagged.sum() == 1
