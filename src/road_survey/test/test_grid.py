"""Accumulation grid statistics and indexing."""

import numpy as np

from road_survey.grid import TerrainGrid, known_bbox


def make():
    return TerrainGrid(origin_x=-5.0, origin_y=-5.0, width=40, height=40,
                       resolution=0.25)


def test_indexing_round_trips():
    g = make()
    ix, iy = g.world_to_cell(0.1, -1.1)
    x, y = g.cell_to_world(ix, iy)
    assert abs(x - 0.1) <= g.res and abs(y + 1.1) <= g.res
    assert g.world_to_cell(g.origin_x, g.origin_y) == (0, 0)


def test_mean_and_std_of_one_cell():
    g = make()
    zs = np.array([1.0, 2.0, 3.0])
    pts = np.column_stack([np.full(3, 0.1), np.full(3, 0.1), zs])
    assert g.add_points(pts) == 3
    ix, iy = g.world_to_cell(0.1, 0.1)
    assert g.count[iy, ix] == 3
    assert np.isclose(g.elevation(min_count=1)[iy, ix], 2.0)
    assert np.isclose(g.within_cell_std(min_count=1)[iy, ix],
                      np.std(zs), atol=1e-5)
    assert np.isclose(g.vertical_extent(min_count=1)[iy, ix], 2.0)


def test_unknown_cells_are_nan():
    g = make()
    g.add_points(np.array([[0.1, 0.1, 1.0]]))
    elev = g.elevation(min_count=1)
    assert np.isnan(elev[0, 0])
    assert np.count_nonzero(np.isfinite(elev)) == 1


def test_min_count_gates_known():
    g = make()
    g.add_points(np.array([[0.1, 0.1, 1.0], [0.1, 0.1, 1.2]]))
    assert not g.known(min_count=3).any()
    assert g.known(min_count=2).sum() == 1


def test_points_outside_are_counted_not_wrapped():
    g = make()
    used = g.add_points(np.array([[100.0, 0.0, 1.0], [0.0, 0.0, 1.0]]))
    assert used == 1
    assert g.dropped == 1
    assert g.count.sum() == 1


def test_coverage_area():
    g = make()
    pts = np.array([[0.1, 0.1, 1.0], [0.6, 0.1, 1.0], [0.1, 0.1, 1.1]])
    g.add_points(pts)
    assert np.isclose(g.coverage_m2(min_count=1), 2 * g.res ** 2)


def test_centered_on_puts_the_point_in_the_middle():
    g = TerrainGrid.centered_on(100.0, -50.0, 40.0, 0.5)
    assert g.width == g.height == 80
    ix, iy = g.world_to_cell(100.0, -50.0)
    assert (ix, iy) == (40, 40)


def test_known_bbox_pads_and_clips():
    known = np.zeros((50, 50), dtype=bool)
    known[20:25, 30:33] = True
    assert known_bbox(known, margin=4) == (16, 29, 26, 37)
    assert known_bbox(known, margin=100) == (0, 50, 0, 50)
    assert known_bbox(np.zeros((5, 5), bool)) is None
