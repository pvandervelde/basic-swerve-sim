import math
import pytest
from typing import Mapping, List, Tuple

# locals
from swerve_controller.errors import InvalidTimeFractionException
from swerve_controller.geometry import PeriodicBoundedCircularSpace
from swerve_controller.profile import SingleVariableCompoundProfile, InvalidTimeFractionException, SingleVariableLinearProfile, SingleVariableMultiPointLinearProfile, SingleVariableSCurveProfile, SingleVariableTrapezoidalProfile

# SingleVariableLinearProfile

def test_should_show_first_derivative_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableLinearProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), end - start, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableLinearProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), end - start, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableLinearProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), (end - start) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), -(end - start) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableLinearProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), (end - start) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), -(end - start) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableLinearProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), (end - start) / 0.01 / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), -(end - start) / 0.01 /0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableLinearProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), (end - start) / 0.01 / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), -(end - start) / 0.01 / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableLinearProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), (start + end) / 2, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableLinearProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), (start + end) / 2, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_linear_profile_and_periodic_valuespace():
    start = 0.5 * math.pi
    end = 1.5 * math.pi
    profile = SingleVariableLinearProfile(start, end, PeriodicBoundedCircularSpace())

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), (start + end) / 2, rel_tol=1e-6, abs_tol=1e-15)

# SingleVariableMultiPointLinearProfile

def test_should_show_first_derivative_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), end - start, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -6 x^2 + 7x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.5, 3.0)

    assert math.isclose(profile.first_derivative_at(0.0), 7.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), -5.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), 1.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    assert math.isclose(profile.first_derivative_at(0.0), 0.25, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), 2.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), 0.9375, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    profile.add_value(0.1, 1.03025)
    profile.add_value(0.2, 1.072)
    profile.add_value(0.4, 1.196)
    profile.add_value(0.5, 1.28125)
    profile.add_value(0.7, 1.50575)
    profile.add_value(0.8, 1.648)
    profile.add_value(0.9, 1.81225)

    assert math.isclose(profile.first_derivative_at(0.0), 0.25, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), 2.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), 0.9375, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -6 x^2 + 7x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.5, 3.0)

    assert math.isclose(profile.second_derivative_at(0.0), -12.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), -12.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), -12.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    assert math.isclose(profile.second_derivative_at(0.0), 1.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 2.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 1.75, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    profile.add_value(0.1, 1.03025)
    profile.add_value(0.2, 1.072)
    profile.add_value(0.4, 1.196)
    profile.add_value(0.5, 1.28125)
    profile.add_value(0.7, 1.50575)
    profile.add_value(0.8, 1.648)
    profile.add_value(0.9, 1.81225)

    assert math.isclose(profile.second_derivative_at(0.0), 1.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 2.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 1.75, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -6 x^2 + 7x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.5, 3.0)

    assert math.isclose(profile.third_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    assert math.isclose(profile.third_derivative_at(0.0), 1.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 1.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 1.5, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    profile.add_value(0.1, 1.03025)
    profile.add_value(0.2, 1.072)
    profile.add_value(0.4, 1.196)
    profile.add_value(0.5, 1.28125)
    profile.add_value(0.7, 1.50575)
    profile.add_value(0.8, 1.648)
    profile.add_value(0.9, 1.81225)

    assert math.isclose(profile.third_derivative_at(0.0), 1.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 1.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 1.5, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), start + (end - start) / 2.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_first_order_multi_point_profile_with_period_valuespace():
    start = 0.5 * math.pi
    end = 1.5 * math.pi
    profile = SingleVariableMultiPointLinearProfile(start, end, PeriodicBoundedCircularSpace())

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), start + (end - start) / 2.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -6 x^2 + 7x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.5, 3.0)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), 3.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.25), 2.375, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75), 2.875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_second_order_multi_point_profile_with_periodic_valuespace():

    # This gives: f(x) = -6 x^2 + 7x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, PeriodicBoundedCircularSpace())
    profile.add_value(0.5, 3.0)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), 3.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.25), 2.375, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75), 2.875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), 1.28125, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(0.25), 1.09765625, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75), 1.57421875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_third_order_multi_point_profile_with_periodic_valuespace():

    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 1.0
    end = 2.0
    profile = SingleVariableMultiPointLinearProfile(start, end, PeriodicBoundedCircularSpace())
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), 1.28125, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(0.25), 1.09765625, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75), 1.57421875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 2.0
    end = 1.0
    profile = SingleVariableMultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    profile.add_value(0.1, 1.03025)
    profile.add_value(0.2, 1.072)
    profile.add_value(0.4, 1.196)
    profile.add_value(0.5, 1.28125)
    profile.add_value(0.7, 1.50575)
    profile.add_value(0.8, 1.648)
    profile.add_value(0.9, 1.81225)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), 1.28125, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(0.25), 1.09765625, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75), 1.57421875, rel_tol=1e-6, abs_tol=1e-15)

# SingleVariableCompoundProfile

def test_should_create_compound_profile_with_single_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 1.0, sub_profile)

    assert profile.value_at(0.0) == sub_profile.value_at(0.0)
    assert profile.value_at(0.5) == sub_profile.value_at(0.5)
    assert profile.value_at(1.0) == sub_profile.value_at(1.0)

def test_should_create_compound_profile_with_multiple_not_connected_profiles():
    profile = SingleVariableCompoundProfile()

    sub_profile_1 = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.2, sub_profile_1)

    sub_profile_2 = SingleVariableLinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.6, sub_profile_2)

    sub_profile_3 = SingleVariableLinearProfile(5.0, 6.0)
    profile.add_profile(0.8, 1.0, sub_profile_3)

    assert profile.value_at(0.0) == sub_profile_1.value_at(0.0)
    assert profile.value_at(0.1) == sub_profile_1.value_at(0.5)
    assert profile.value_at(0.2) == sub_profile_1.value_at(1.0)

    assert profile.value_at(0.3) == (sub_profile_1.value_at(1.0) + sub_profile_2.value_at(0.0))/2.0

    assert profile.value_at(0.4) == sub_profile_2.value_at(0.0)
    assert profile.value_at(0.5) == sub_profile_2.value_at(0.5)
    assert profile.value_at(0.6) == sub_profile_2.value_at(1.0)

    assert profile.value_at(0.7) == (sub_profile_2.value_at(1.0) + sub_profile_3.value_at(0.0))/2.0

    assert profile.value_at(0.8) == sub_profile_3.value_at(0.0)
    assert profile.value_at(0.9) == sub_profile_3.value_at(0.5)
    assert profile.value_at(1.0) == sub_profile_3.value_at(1.0)

def test_should_create_compound_profile_with_multiple_connected_profiles():
    profile = SingleVariableCompoundProfile()

    sub_profile_1 = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = SingleVariableLinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    sub_profile_3 = SingleVariableLinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    assert profile.value_at(0.0) == sub_profile_1.value_at(0.0)
    assert profile.value_at(0.2) == sub_profile_1.value_at(0.5)

    assert profile.value_at(0.4) == sub_profile_2.value_at(0.0)
    assert profile.value_at(0.55) == sub_profile_2.value_at(0.5)

    assert profile.value_at(0.7) == sub_profile_3.value_at(0.0)
    assert profile.value_at(0.85) == sub_profile_3.value_at(0.5)
    assert profile.value_at(1.0) == sub_profile_3.value_at(1.0)

def test_should_create_compound_profile_with_multiple_out_of_order_profiles():
    profile = SingleVariableCompoundProfile()

    sub_profile_3 = SingleVariableLinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    sub_profile_1 = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = SingleVariableLinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    assert profile.value_at(0.0) == sub_profile_1.value_at(0.0)
    assert profile.value_at(0.2) == sub_profile_1.value_at(0.5)

    assert profile.value_at(0.4) == sub_profile_2.value_at(0.0)
    assert profile.value_at(0.55) == sub_profile_2.value_at(0.5)

    assert profile.value_at(0.7) == sub_profile_3.value_at(0.0)
    assert profile.value_at(0.85) == sub_profile_3.value_at(0.5)
    assert profile.value_at(1.0) == sub_profile_3.value_at(1.0)

def test_should_throw_error_with_multiple_overlapping_sub_profiles_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile_1 = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.40, sub_profile_1)

    sub_profile_3 = SingleVariableLinearProfile(5.0, 6.0)
    profile.add_profile(0.60, 1.0, sub_profile_3)

    sub_profile_2 = SingleVariableLinearProfile(3.0, 4.0)

    with pytest.raises(InvalidTimeFractionException):
        profile.add_profile(0.35, 0.70, sub_profile_2)

def test_should_show_value_at_with_no_sub_profile_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    assert math.isclose(profile.value_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_single_sub_profile_that_covers_start_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.2, sub_profile)

    assert profile.value_at(0.0) == sub_profile.value_at(0.0)
    assert profile.value_at(0.1) == sub_profile.value_at(0.5)
    assert profile.value_at(0.2) == sub_profile.value_at(1.0)

    assert profile.value_at(0.4) == sub_profile.value_at(1.0)
    assert profile.value_at(0.6) == sub_profile.value_at(1.0)
    assert profile.value_at(0.8) == sub_profile.value_at(1.0)
    assert profile.value_at(1.0) == sub_profile.value_at(1.0)

def test_should_show_value_at_with_single_sub_profile_that_covers_end_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.8, 1.0, sub_profile)

    assert profile.value_at(0.0) == sub_profile.value_at(0.0)
    assert profile.value_at(0.2) == sub_profile.value_at(0.0)
    assert profile.value_at(0.4) == sub_profile.value_at(0.0)
    assert profile.value_at(0.6) == sub_profile.value_at(0.0)

    assert profile.value_at(0.8) == sub_profile.value_at(0.0)
    assert profile.value_at(0.9) == sub_profile.value_at(0.5)
    assert profile.value_at(1.0) == sub_profile.value_at(1.0)

def test_should_show_first_derivative_at_with_single_sub_profile_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 1.0, sub_profile)

    assert profile.first_derivative_at(0.0) == sub_profile.first_derivative_at(0.0)
    assert profile.first_derivative_at(0.5) == sub_profile.first_derivative_at(0.5)
    assert profile.first_derivative_at(1.0) == sub_profile.first_derivative_at(1.0)

def test_should_show_first_derivative_at_with_multiple_sub_profiles_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile_1 = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = SingleVariableLinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    sub_profile_3 = SingleVariableLinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    assert profile.first_derivative_at(0.0) == sub_profile_1.first_derivative_at(0.0)
    assert profile.first_derivative_at(0.2) == sub_profile_1.first_derivative_at(0.5)

    assert profile.first_derivative_at(0.4) == sub_profile_2.first_derivative_at(0.0)
    assert profile.first_derivative_at(0.55) == sub_profile_2.first_derivative_at(0.5)

    assert profile.first_derivative_at(0.7) == sub_profile_3.first_derivative_at(0.0)
    assert profile.first_derivative_at(0.85) == sub_profile_3.first_derivative_at(0.5)
    assert profile.first_derivative_at(1.0) == sub_profile_3.first_derivative_at(1.0)

def test_should_show_second_derivative_at_with_single_sub_profile_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 1.0, sub_profile)

    assert profile.second_derivative_at(0.0) == sub_profile.second_derivative_at(0.0)
    assert profile.second_derivative_at(0.5) == sub_profile.second_derivative_at(0.5)
    assert profile.second_derivative_at(1.0) == sub_profile.second_derivative_at(1.0)

def test_should_show_second_derivative_at_with_multiple_sub_profiles_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile_1 = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = SingleVariableLinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    sub_profile_3 = SingleVariableLinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    assert profile.second_derivative_at(0.0) == sub_profile_1.second_derivative_at(0.0)
    assert profile.second_derivative_at(0.2) == sub_profile_1.second_derivative_at(0.5)

    assert profile.second_derivative_at(0.4) == sub_profile_2.second_derivative_at(0.0)
    assert profile.second_derivative_at(0.55) == sub_profile_2.second_derivative_at(0.5)

    assert profile.second_derivative_at(0.7) == sub_profile_3.second_derivative_at(0.0)
    assert profile.second_derivative_at(0.85) == sub_profile_3.second_derivative_at(0.5)
    assert profile.second_derivative_at(1.0) == sub_profile_3.second_derivative_at(1.0)

def test_should_show_third_derivative_at_with_single_sub_profile_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 1.0, sub_profile)

    assert profile.third_derivative_at(0.0) == sub_profile.third_derivative_at(0.0)
    assert profile.third_derivative_at(0.5) == sub_profile.third_derivative_at(0.5)
    assert profile.third_derivative_at(1.0) == sub_profile.third_derivative_at(1.0)

def test_should_show_third_derivative_at_with_multiple_sub_profiles_in_compound_profile():
    profile = SingleVariableCompoundProfile()

    sub_profile_1 = SingleVariableLinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = SingleVariableLinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    sub_profile_3 = SingleVariableLinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    assert profile.third_derivative_at(0.0) == sub_profile_1.third_derivative_at(0.0)
    assert profile.third_derivative_at(0.2) == sub_profile_1.third_derivative_at(0.5)

    assert profile.third_derivative_at(0.4) == sub_profile_2.third_derivative_at(0.0)
    assert profile.third_derivative_at(0.55) == sub_profile_2.third_derivative_at(0.5)

    assert profile.third_derivative_at(0.7) == sub_profile_3.third_derivative_at(0.0)
    assert profile.third_derivative_at(0.85) == sub_profile_3.third_derivative_at(0.5)
    assert profile.third_derivative_at(1.0) == sub_profile_3.third_derivative_at(1.0)

# SingleVariableMultiPointLinearProfile

# SingleVariableTrapezoidalProfile

def test_should_show_first_derivative_at_with_increasing_trapezoidal_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/3), 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/3), 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/6), 0.75 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/6), 0.75 * (end - start), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_decreasing_trapezoidal_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableTrapezoidalProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/3), 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/3), 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/6), 0.75 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/6), 0.75 * (end - start), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_increasing_trapezoidal_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 1.5 * (end - start) * 3, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), -1.5 * (end - start) * 3, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/3), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/3), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/6), 1.5 * (end - start) * 3, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/6), -1.5 * (end - start) * 3, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_decreasing_trapezoidal_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableTrapezoidalProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 1.5 * (end - start) * 3, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), -1.5 * (end - start) * 3, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/3), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/3), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/6), 1.5 * (end - start) * 3, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/6), -1.5 * (end - start) * 3, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_increasing_trapezoidal_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), (1.5 * (end - start) * 3) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), (1.5 * (end - start) * 3) / 0.01, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/3), -(1.5 * (end - start) * 3) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/3), -(1.5 * (end - start) * 3) / 0.01, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/6), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/6), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_decreasing_trapezoidal_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableTrapezoidalProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), (1.5 * (end - start) * 3) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), (1.5 * (end - start) * 3) / 0.01, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/3), -(1.5 * (end - start) * 3) / 0.01, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/3), -(1.5 * (end - start) * 3) / 0.01, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/6), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/6), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_trapezoidal_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableTrapezoidalProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/3), start + 0.5 * 1/3 * 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/3), start + 1.5 * 1/3 * 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/6), start + 0.5 * 4.5 * (end - start) * 1/6 * 1/6, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), start + 1.0 * 1/3 * 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/6), start + 1.5 * 1/3 * 1.5 * (end - start) + (1.5 * (end - start) * 1/6 - 0.5 * 4.5 * (end - start) * 1/6 * 1/6), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_trapezoidal_profile_with_periodic_valuespace():
    start = 0.5 * math.pi
    end = 1.5 * math.pi
    profile = SingleVariableTrapezoidalProfile(start, end, PeriodicBoundedCircularSpace())

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/3), start + 0.5 * 1/3 * 1.5 * (-0.5 * math.pi - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/3), start + 1.5 * 1/3 * 1.5 * (-0.5 * math.pi - start), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/6), start + 0.5 * 4.5 * (-0.5 * math.pi - start) * 1/6 * 1/6, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), start + 1.0 * 1/3 * 1.5 * (-0.5 * math.pi - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/6), start + 1.5 * 1/3 * 1.5 * (-0.5 * math.pi - start) + (1.5 * (-0.5 * math.pi - start) * 1/6 - 0.5 * 4.5 * (-0.5 * math.pi - start) * 1/6 * 1/6), rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_decreasing_trapezoidal_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableTrapezoidalProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/3), start + 0.5 * 1/3 * 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/3), start + 1.5 * 1/3 * 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/6), start + 0.5 * 4.5 * (end - start) * 1/6 * 1/6, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), start + 1.0 * 1/3 * 1.5 * (end - start), rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/6), start + 1.5 * 1/3 * 1.5 * (end - start) + (1.5 * (end - start) * 1/6 - 0.5 * 4.5 * (end - start) * 1/6 * 1/6), rel_tol=1e-6, abs_tol=1e-15)

# SingleVariableSCurveProfile

def test_should_show_first_derivative_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/8), 0.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/8), 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/8), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(4/8), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/8), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(6/8), 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/8), 0.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(8/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/16), 0.5 * 51.2 * 1/256, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/16), 1.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/16), 51.2 * 1/128 - 0.5 * 51.2 * 1/256 + 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/16), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(9/16), 2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(11/16), 51.2 * 1/128 - 0.5 * 51.2 * 1/256 + 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(13/16), 1.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(15/16), 0.5 * 51.2 * 1/256, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/8), -0.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(2/8), -1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/8), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(4/8), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/8), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(6/8), -1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/8), -0.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(8/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.first_derivative_at(1/16), -0.5 * 51.2 * 1/256, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(3/16), -1.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(5/16), -51.2 * 1/128 + 0.5 * 51.2 * 1/256 - 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(7/16), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(9/16), -2.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(11/16), -51.2 * 1/128 + 0.5 * 51.2 * 1/256 - 1.5 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(13/16), -1.0 * 51.2 * 1/64, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(15/16), -0.5 * 51.2 * 1/256, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/8), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/8), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(4/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(6/8), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/8), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(8/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/16), 51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/16), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/16), 51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(9/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(11/16), -51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(13/16), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(15/16), -51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/8), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(2/8), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(4/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(6/8), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/8), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(8/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.second_derivative_at(1/16), -51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(3/16), -51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(5/16), -51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(7/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(9/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(11/16), 51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(13/16), 51.2 * 1/8, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(15/16), 51.2 * 1/16, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 51.2, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/8), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(4/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/8), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(6/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/8), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(8/8), 51.2, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/16), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/16), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(9/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(11/16), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(13/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(15/16), 51.2, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), -51.2, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(2/8), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(4/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/8), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(6/8), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/8), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(8/8), -51.2, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.third_derivative_at(1/16), -51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(3/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(5/16), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(7/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(9/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(11/16), 51.2, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(13/16), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(15/16), -51.2, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_scurve_profile():
    start = 1.0
    end = 2.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/8), 1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/8), 1.0 * 51.2 * 1/512 + 1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(3/8), 3.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(4/8), 5.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/8), 7.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(6/8), (8 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(7/8), (9 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(8/8), end, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_scurve_profile_with_periodic_valuespace():
    start = 0.5 * math.pi
    end = 1.5 * math.pi
    profile = SingleVariableSCurveProfile(start, end, PeriodicBoundedCircularSpace())

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/8), 1/6 * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/8), (1.0 + 1/6) * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(3/8), 3.0 * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(4/8), 5.0 * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/8), 7.0 * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(6/8), (8 + 5/6) * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(7/8), (9 + 5/6) * 51.2 * -math.pi * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(8/8), -0.5 * math.pi, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_decreasing_scurve_profile():
    start = 2.0
    end = 1.0
    profile = SingleVariableSCurveProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(profile.value_at(1/8), -1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(2/8), -1.0 * 51.2 * 1/512 - 1/6 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(3/8), -3.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(4/8), -5.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(5/8), -7.0 * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(6/8), -(8 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(7/8), -(9 + 5/6) * 51.2 * 1/512 + start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(8/8), end, rel_tol=1e-6, abs_tol=1e-15)
