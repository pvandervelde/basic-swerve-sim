import math
import pytest
from typing import Mapping, List, Tuple

# locals
from swerve_controller.errors import InvalidTimeFractionException
from swerve_controller.geometry import PeriodicBoundedCircularSpace
from swerve_controller.profile import InvalidTimeFractionException, SingleVariableLinearProfile, SingleVariableMultiPointLinearProfile, SingleVariableSCurveProfile, SingleVariableTrapezoidalProfile

# SingleVariableLinearProfile

def test_should_show_first_derivative_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.first_derivative_at(0.0), (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5 * end_time), (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_decreasing_linear_profile():
    start = 3.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.first_derivative_at(0.0), (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5 * end_time), (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.second_derivative_at(0.0), (end - start) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), -(end - start) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.second_derivative_at(0.0), (end - start) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), -(end - start) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.third_derivative_at(0.0), (end - start) / 0.01 / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), -(end - start) / 0.01 /0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.third_derivative_at(0.0), (end - start) / 0.01 / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), -(end - start) / 0.01 / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), (start + end) / 2, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), (start + end) / 2, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_linear_profile_and_periodic_valuespace():
    start = 0.5 * math.pi
    end = 1.5 * math.pi
    end_time = 2.0
    profile = SingleVariableLinearProfile(start, end, end_time, PeriodicBoundedCircularSpace())

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), (start + end) / 2, rel_tol=1e-6, abs_tol=1e-15)

# SingleVariableMultiPointLinearProfile

def test_should_show_first_derivative_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.first_derivative_at(0.0), (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5 * end_time), (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -1.5 x^2 + 3.5x + 1 -> f'(x) = -3x + 3.5
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.5 * end_time, 3.0)

    assert math.isclose(profile.first_derivative_at(0.0), 3.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), -2.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5 * end_time), 0.5, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.03125 * x^3 + 0.125 * x^2 + 0.125 * x + 1 -> f'(x) = 0.09375 * x^2 + 0.25 * x + 0.125
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.3 * end_time, 1.12675)
    profile.add_value(0.6 * end_time, 1.384)

    assert math.isclose(profile.first_derivative_at(0.0), 0.125, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), 1.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5 * end_time), 0.46875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.03125 * x^3 + 0.125 * x^2 + 0.125 * x + 1 -> f'(x) = 0.09375 * x^2 + 0.25 * x + 0.125
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.3 * end_time, 1.12675)
    profile.add_value(0.6 * end_time, 1.384)

    profile.add_value(0.1 * end_time, 1.03025)
    profile.add_value(0.2 * end_time, 1.072)
    profile.add_value(0.4 * end_time, 1.196)
    profile.add_value(0.5 * end_time, 1.28125)
    profile.add_value(0.7 * end_time, 1.50575)
    profile.add_value(0.8 * end_time, 1.648)
    profile.add_value(0.9 * end_time, 1.81225)

    assert math.isclose(profile.first_derivative_at(0.0), 0.125, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), 1.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5 * end_time), 0.46875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -1.5 x^2 + 3.5x + 1 -> f'(x) = -3x + 3.5 -> f''(x) = -3
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.5 * end_time, 3.0)

    assert math.isclose(profile.second_derivative_at(0.0), -3.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), -3.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5 * end_time), -3.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.03125 * x^3 + 0.125 * x^2 + 0.125 * x + 1 -> f'(x) = 0.09375 * x^2 + 0.25 * x + 0.125 -> f''(x) = 0.1875 * x + 0.25
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.3 * end_time, 1.12675)
    profile.add_value(0.6 * end_time, 1.384)

    assert math.isclose(profile.second_derivative_at(0.0), 0.25, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), 0.625, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5 * end_time), 0.4375, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.03125 * x^3 + 0.125 * x^2 + 0.125 * x + 1 -> f'(x) = 0.09375 * x^2 + 0.25 * x + 0.125 -> f''(x) = 0.1875 * x + 0.25
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.3 * end_time, 1.12675)
    profile.add_value(0.6 * end_time, 1.384)

    profile.add_value(0.1 * end_time, 1.03025)
    profile.add_value(0.2 * end_time, 1.072)
    profile.add_value(0.4 * end_time, 1.196)
    profile.add_value(0.5 * end_time, 1.28125)
    profile.add_value(0.7 * end_time, 1.50575)
    profile.add_value(0.8 * end_time, 1.648)
    profile.add_value(0.9 * end_time, 1.81225)

    assert math.isclose(profile.second_derivative_at(0.0), 0.25, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), 0.625, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5 * end_time), 0.4375, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.third_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -1.5 x^2 + 3.5x + 1 -> f'(x) = -3x + 3.5 -> f''(x) = -3
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.5 * end_time, 3.0)

    assert math.isclose(profile.third_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.03125 * x^3 + 0.125 * x^2 + 0.125 * x + 1 -> f'(x) = 0.09375 * x^2 + 0.25 * x + 0.125 -> f''(x) = 0.1875 * x + 0.25 -> f'''(x) = 0.1875
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.3 * end_time, 1.12675)
    profile.add_value(0.6 * end_time, 1.384)

    assert math.isclose(profile.third_derivative_at(0.0), 0.1875, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), 0.1875, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5 * end_time), 0.1875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.03125 * x^3 + 0.125 * x^2 + 0.125 * x + 1 -> f'(x) = 0.09375 * x^2 + 0.25 * x + 0.125 -> f''(x) = 0.1875 * x + 0.25 -> f'''(x) = 0.1875
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.3 * end_time, 1.12675)
    profile.add_value(0.6 * end_time, 1.384)

    profile.add_value(0.1 * end_time, 1.03025)
    profile.add_value(0.2 * end_time, 1.072)
    profile.add_value(0.4 * end_time, 1.196)
    profile.add_value(0.5 * end_time, 1.28125)
    profile.add_value(0.7 * end_time, 1.50575)
    profile.add_value(0.8 * end_time, 1.648)
    profile.add_value(0.9 * end_time, 1.81225)

    assert math.isclose(profile.third_derivative_at(0.0), 0.1875, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), 0.1875, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5 * end_time), 0.1875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), start + (end - start) / 2.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_first_order_multi_point_profile_with_period_valuespace():
    start = 0.5 * math.pi
    end = 1.5 * math.pi
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time, coordinate_space=PeriodicBoundedCircularSpace())

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), start + (end - start) / 2.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -1.5 x^2 + 3.5x + 1
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.5 * end_time, 3.0)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), 3.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.25 * end_time), 2.375, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75 * end_time), 2.875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_second_order_multi_point_profile_with_periodic_valuespace():

    # This gives: f(x) = -1.5 x^2 + 3.5x + 1
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time, coordinate_space=PeriodicBoundedCircularSpace())
    profile.add_value(0.5 * end_time, 3.0)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), 3.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.25 * end_time), 2.375, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75 * end_time), 2.875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.03125 * x^3 + 0.125 * x^2 + 0.125 * x + 1
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.3 * end_time, 1.12675)
    profile.add_value(0.6 * end_time, 1.384)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), 1.28125, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(0.25 * end_time), 1.09765625, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75 * end_time), 1.57421875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_third_order_multi_point_profile_with_periodic_valuespace():

    # This gives: f(x) = 0.03125 * x^3 + 0.125 * x^2 + 0.125 * x + 1
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time, coordinate_space=PeriodicBoundedCircularSpace())
    profile.add_value(0.3 * end_time, 1.12675)
    profile.add_value(0.6 * end_time, 1.384)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), 1.28125, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(0.25 * end_time), 1.09765625, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75 * end_time), 1.57421875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.03125 * x^3 + 0.125 * x^2 + 0.125 * x + 1
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, end_time=end_time)
    profile.add_value(0.3 * end_time, 1.12675)
    profile.add_value(0.6 * end_time, 1.384)

    profile.add_value(0.1 * end_time, 1.03025)
    profile.add_value(0.2 * end_time, 1.072)
    profile.add_value(0.4 * end_time, 1.196)
    profile.add_value(0.5 * end_time, 1.28125)
    profile.add_value(0.7 * end_time, 1.50575)
    profile.add_value(0.8 * end_time, 1.648)
    profile.add_value(0.9 * end_time, 1.81225)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), 1.28125, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(0.25 * end_time), 1.09765625, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75 * end_time), 1.57421875, rel_tol=1e-6, abs_tol=1e-15)

# SingleVariableMultiPointLinearProfile

# SingleVariableTrapezoidalProfile

def test_should_show_first_derivative_at_with_increasing_trapezoidal_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end, end_time=end_time)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/3 * end_time), 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/3 * end_time), 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/6 * end_time), 0.75 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5 * end_time), 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/6 * end_time), 0.75 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_decreasing_trapezoidal_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end, end_time=end_time)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/3 * end_time), 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/3 * end_time), 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/6 * end_time), 0.75 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5 * end_time), 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/6 * end_time), 0.75 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_increasing_trapezoidal_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end, end_time=end_time)

    assert math.isclose(profile.second_derivative_at(0.0), (1.5 * (end - start) / end_time) / (1/3 * end_time), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), -(1.5 * (end - start) / end_time) / (1/3 * end_time), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/3 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/3 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/6 * end_time), (1.5 * (end - start) / end_time) / (1/3 * end_time), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/6 * end_time), -(1.5 * (end - start) / end_time) / (1/3 * end_time), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_decreasing_trapezoidal_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end, end_time=end_time)

    assert math.isclose(profile.second_derivative_at(0.0), (1.5 * (end - start) / end_time) / (1/3 * end_time), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), -(1.5 * (end - start) / end_time) / (1/3 * end_time), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/3 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/3 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/6 * end_time), (1.5 * (end - start) / end_time) / (1/3 * end_time), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/6 * end_time), -(1.5 * (end - start) / end_time) / (1/3 * end_time), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_increasing_trapezoidal_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end, end_time=end_time)

    assert math.isclose(profile.third_derivative_at(0.0), (1.5 * (end - start) / end_time) / (1/3 * end_time) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), (1.5 * (end - start) / end_time) / (1/3 * end_time)/ 0.01, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/3 * end_time), -(1.5 * (end - start) / end_time) / (1/3 * end_time) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/3 * end_time), -(1.5 * (end - start) / end_time) / (1/3 * end_time) / 0.01, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/6 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/6 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_decreasing_trapezoidal_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end, end_time=end_time)

    assert math.isclose(profile.third_derivative_at(0.0), (1.5 * (end - start) / end_time) / (1/3 * end_time) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), (1.5 * (end - start) / end_time) / (1/3 * end_time) / 0.01, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/3 * end_time), -(1.5 * (end - start) / end_time) / (1/3 * end_time) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/3 * end_time), -(1.5 * (end - start) / end_time) / (1/3 * end_time) / 0.01, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/6 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/6 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_trapezoidal_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end, end_time=end_time)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/3 * end_time), start + 0.5 * end_time/3 * 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/3 * end_time), start + 1.5 * end_time/3 * 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/6 * end_time), start + 0.5 * 1.5 * 3 / end_time * (end - start) / end_time * (end_time/6) * (end_time/6), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), start + 1.0 * end_time/3 * 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/6 * end_time), start + 1.5 * end_time/3 * 1.5 * (end - start) / end_time + (1.5 * (end - start) / end_time * end_time/6 - 0.5 * 4.5/end_time * (end - start) / end_time * end_time/6 * end_time/6), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_trapezoidal_profile_with_periodic_valuespace():
    start = 0.5 * math.pi
    end = 1.5 * math.pi
    end_time = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end, end_time, PeriodicBoundedCircularSpace())

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/3 * end_time), start + 0.5 * 1/3 * 1.5 * (-0.5 * math.pi - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/3 * end_time), start + 1.5 * 1/3 * 1.5 * (-0.5 * math.pi - start), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/6 * end_time), start + 0.5 * 4.5 * (-0.5 * math.pi - start) * 1/6 * 1/6, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), start + 1.0 * 1/3 * 1.5 * (-0.5 * math.pi - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/6 * end_time), start + 1.5 * 1/3 * 1.5 * (-0.5 * math.pi - start) + (1.5 * (-0.5 * math.pi - start) * 1/6 - 0.5 * 4.5 * (-0.5 * math.pi - start) * 1/6 * 1/6), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_decreasing_trapezoidal_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end, end_time=end_time)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/3 * end_time), start + 0.5 * end_time/3 * 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/3 * end_time), start + 1.5 * end_time/3 * 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/6 * end_time), start + 0.5 * 1.5 * 3 / end_time * (end - start) / end_time * (end_time/6) * (end_time/6), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5 * end_time), start + 1.0 * end_time/3 * 1.5 * (end - start) / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/6 * end_time), start + 1.5 * end_time/3 * 1.5 * (end - start) / end_time + (1.5 * (end - start) / end_time * end_time/6 - 0.5 * 4.5/end_time * (end - start) / end_time * end_time/6 * end_time/6), rel_tol=1e-6, abs_tol=1e-15)

# SingleVariableSCurveProfile

def test_should_show_first_derivative_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableSCurveProfile(start, end, end_time=end_time)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/8 * end_time), 0.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/8 * end_time), 1.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/8 * end_time), 2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(4/8 * end_time), 2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/8 * end_time), 2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(6/8 * end_time), 1.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/8 * end_time), 0.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(8/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/16 * end_time), 0.5 * 51.2 * 1/256 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/16 * end_time), 1.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/16 * end_time), 51.2 * 1/128 / end_time - 0.5 * 51.2 * 1/256 / end_time + 1.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/16 * end_time), 2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(9/16 * end_time), 2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(11/16 * end_time), 51.2 * 1/128 / end_time - 0.5 * 51.2 * 1/256 / end_time + 1.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(13/16 * end_time), 1.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(15/16 * end_time), 0.5 * 51.2 * 1/256 / end_time, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableSCurveProfile(start, end, end_time=end_time)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/8 * end_time), -0.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/8 * end_time), -1.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/8 * end_time), -2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(4/8 * end_time), -2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/8 * end_time), -2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(6/8 * end_time), -1.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/8 * end_time), -0.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(8/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/16 * end_time), -0.5 * 51.2 * 1/256 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/16 * end_time), -1.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/16 * end_time), -51.2 * 1/128 / end_time + 0.5 * 51.2 * 1/256 / end_time - 1.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/16 * end_time), -2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(9/16 * end_time), -2.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(11/16 * end_time), -51.2 * 1/128 / end_time + 0.5 * 51.2 * 1/256 / end_time - 1.5 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(13/16 * end_time), -1.0 * 51.2 * 1/64 / end_time, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(15/16 * end_time), -0.5 * 51.2 * 1/256 / end_time, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableSCurveProfile(start, end, end_time=end_time)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/8 * end_time), 51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/8 * end_time), 51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(4/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(6/8 * end_time), -51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/8 * end_time), -51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(8/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/16 * end_time), 51.2 * 1/16 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/16 * end_time), 51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/16 * end_time), 51.2 * 1/16 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(9/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(11/16 * end_time), -51.2 * 1/16 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(13/16 * end_time), -51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(15/16 * end_time), -51.2 * 1/16 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableSCurveProfile(start, end, end_time=end_time)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/8 * end_time), -51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/8 * end_time), -51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(4/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(6/8 * end_time), 51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/8 * end_time), 51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(8/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/16 * end_time), -51.2 * 1/16 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/16 * end_time), -51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/16 * end_time), -51.2 * 1/16 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(9/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(11/16 * end_time), 51.2 * 1/16 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(13/16 * end_time), 51.2 * 1/8 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(15/16 * end_time), 51.2 * 1/16 / math.pow(end_time, 2.0), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableSCurveProfile(start, end, end_time=end_time)

    assert math.isclose(profile.third_derivative_at(0.0), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/8 * end_time), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(4/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/8 * end_time), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(6/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/8 * end_time), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(8/8 * end_time), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/16 * end_time), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/16 * end_time), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(9/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(11/16 * end_time), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(13/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(15/16 * end_time), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableSCurveProfile(start, end, end_time=end_time)

    assert math.isclose(profile.third_derivative_at(0.0), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(end_time), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/8 * end_time), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(4/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/8 * end_time), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(6/8 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/8 * end_time), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(8/8 * end_time), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/16 * end_time), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/16 * end_time), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(9/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(11/16 * end_time), 51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(13/16 * end_time), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(15/16 * end_time), -51.2 / math.pow(end_time, 3.0), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    end_time = 2.0
    profile = SingleVariableSCurveProfile(start, end, end_time=end_time)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/8 * end_time), 1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/8 * end_time), 1.0 * 51.2 * 1/512 + 1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(3/8 * end_time), 3.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(4/8 * end_time), 5.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/8 * end_time), 7.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(6/8 * end_time), (8 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(7/8 * end_time), (9 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(8/8 * end_time), end, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_scurve_profile_with_periodic_valuespace():
    start = 0.5 * math.pi
    end = 1.5 * math.pi
    end_time = 2.0
    profile = SingleVariableSCurveProfile(start, end, end_time, PeriodicBoundedCircularSpace())

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/8 * end_time), 1/6 * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/8 * end_time), (1.0 + 1/6) * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(3/8 * end_time), 3.0 * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(4/8 * end_time), 5.0 * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/8 * end_time), 7.0 * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(6/8 * end_time), (8 + 5/6) * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(7/8 * end_time), (9 + 5/6) * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(8/8 * end_time), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    end_time = 2.0
    profile = SingleVariableSCurveProfile(start, end, end_time=end_time)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(end_time), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/8 * end_time), -1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/8 * end_time), -1.0 * 51.2 * 1/512 - 1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(3/8 * end_time), -3.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(4/8 * end_time), -5.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/8 * end_time), -7.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(6/8 * end_time), -(8 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(7/8 * end_time), -(9 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(8/8 * end_time), end, rel_tol=1e-6, abs_tol=1e-15)
