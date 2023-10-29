from math import inf
import math
from matplotlib.pylab import normal
import pytest

from swerve_controller.geometry import CircularSpace, LinearSpace

# LinearSpace

def test_linear_space_smallest_distance_between_values():
    space = LinearSpace()
    assert math.isclose(space.smallest_distance_between_values(0, 0), 0, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(0, 1), 1, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(1, 0), -1, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(-1, 1), 2, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(1, -1), -2, rel_tol=1e-6, abs_tol=1e-6)

    assert math.isinf(space.smallest_distance_between_values(0, math.inf))
    assert math.isinf(space.smallest_distance_between_values(-math.inf, 0))

    assert math.isinf(space.smallest_distance_between_values(math.inf, 0))
    assert math.isinf(space.smallest_distance_between_values(0, math.inf))

    assert math.isinf(space.smallest_distance_between_values(-math.inf, math.inf))

def test_linear_space_distances_between_values():
    space = LinearSpace()
    assert space.distances_between_values(0, 0) == [0]
    assert space.distances_between_values(0, 1) == [1]
    assert space.distances_between_values(1, 0) == [-1]
    assert space.distances_between_values(-1, 1) == [2]
    assert space.distances_between_values(1, -1) == [-2]

    assert space.distances_between_values(0, math.inf) == [ math.inf ]
    assert space.distances_between_values(-math.inf, 0) == [ math.inf ]

    assert space.distances_between_values(math.inf, 0) == [ -math.inf ]
    assert space.distances_between_values(0, -math.inf) == [ -math.inf ]

    assert space.distances_between_values(-math.inf, math.inf) == [ math.inf ]

def test_linear_space_normalize_value():
    space = LinearSpace()
    assert math.isclose(space.normalize_value(0), 0, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(1), 1, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(-1), -1, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(100), 100, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(-100), -100, rel_tol=1e-6, abs_tol=1e-6)

    assert space.normalize_value(math.inf) == math.inf
    assert space.normalize_value(-math.inf) == -math.inf


# CircularSpace

def test_circular_space_smallest_distance_between_values():
    space = CircularSpace()
    assert math.isclose(space.smallest_distance_between_values(0, 0), 0, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(0, math.pi), math.pi, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(math.pi, 0), math.pi, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(-math.pi, math.pi), 0, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(math.pi, -math.pi), 0, rel_tol=1e-6, abs_tol=1e-6)

    assert math.isclose(space.smallest_distance_between_values(0, 2 * math.pi), 0, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(0, 4 * math.pi), 0, rel_tol=1e-6, abs_tol=1e-6)

    assert math.isclose(space.smallest_distance_between_values(math.pi, 3 * math.pi), 0, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.smallest_distance_between_values(11 * math.pi, 20 * math.pi), math.pi, rel_tol=1e-6, abs_tol=1e-6)

    assert math.isclose(space.smallest_distance_between_values(0.25 * math.pi, 0.5 * math.pi), 0.25 * math.pi, rel_tol=1e-6, abs_tol=1e-6)

def test_circular_space_distances_between_values():
    space = CircularSpace()
    assert space.distances_between_values(0, 0) == [0, 2 * math.pi]
    assert space.distances_between_values(0, math.pi) == [math.pi, -math.pi]
    assert space.distances_between_values(math.pi, 0) == [-math.pi, math.pi]
    assert space.distances_between_values(-math.pi, math.pi) == [0, 2 * math.pi]
    assert space.distances_between_values(math.pi, -math.pi) == [0, 2 * math.pi]

def test_circular_space_normalize_value():
    space = CircularSpace()
    assert math.isclose(space.normalize_value(0), 0, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(math.pi), math.pi, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(-math.pi), math.pi, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(3 * math.pi), math.pi, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(-3 * math.pi), math.pi, rel_tol=1e-6, abs_tol=1e-6)

    assert math.isclose(space.normalize_value(0.25 * math.pi), 0.25 * math.pi, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(-0.25 * math.pi), -0.25 * math.pi, rel_tol=1e-6, abs_tol=1e-6)

    assert math.isclose(space.normalize_value(0.75 * math.pi), 0.75 * math.pi, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(-0.75 * math.pi), -0.75 * math.pi, rel_tol=1e-6, abs_tol=1e-6)

    assert math.isclose(space.normalize_value(-1.5 * math.pi), 0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(space.normalize_value(1.5 * math.pi), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-6)

