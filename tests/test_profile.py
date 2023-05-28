import math
import pytest
from typing import Mapping, List, Tuple

# locals
from swerve_controller.errors import InvalidTimeFractionException
from swerve_controller.profile import CompoundProfile, InvalidTimeFractionException, LinearProfile, MultiPointLinearProfile

# LinearProfile

def test_should_show_first_derivative_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    profile = LinearProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), end - start, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    profile = LinearProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), end - start, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_inflection_points_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    profile = LinearProfile(start, end)

    points = profile.inflection_points()

    assert len(points) == 2

    assert math.isclose(points[0].time_fraction, 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[0].value, start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[0].first_derivative, end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[0].second_derivative, 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[0].third_derivative, 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(points[1].time_fraction, 1.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[1].value, end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[1].first_derivative, end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[1].second_derivative, 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[1].third_derivative, 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_inflection_points_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    profile = LinearProfile(start, end)

    points = profile.inflection_points()

    assert len(points) == 2

    assert math.isclose(points[0].time_fraction, 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[0].value, start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[0].first_derivative, end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[0].second_derivative, 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[0].third_derivative, 0.0, rel_tol=1e-6, abs_tol=1e-15)

    assert math.isclose(points[1].time_fraction, 1.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[1].value, end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[1].first_derivative, end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[1].second_derivative, 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(points[1].third_derivative, 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    profile = LinearProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    profile = LinearProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    profile = LinearProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    profile = LinearProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_increasing_linear_profile():
    start = 1.0
    end = 2.0
    profile = LinearProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), (start + end) / 2, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_decreasing_linear_profile():
    start = 2.0
    end = 1.0
    profile = LinearProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), (start + end) / 2, rel_tol=1e-6, abs_tol=1e-15)

# MultiPointLinearProfile

def test_should_show_first_derivative_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    profile = MultiPointLinearProfile(start, end)

    assert math.isclose(profile.first_derivative_at(0.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), end - start, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -6 x^2 + 7x + 1
    start = 1.0
    end = 2.0
    profile = MultiPointLinearProfile(start, end)
    profile.add_value(0.5, 3.0)

    assert math.isclose(profile.first_derivative_at(0.0), 7.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), -5.0 - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), 1.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 2.0
    end = 1.0
    profile = MultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    assert math.isclose(profile.first_derivative_at(0.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), end - start, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_first_derivative_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 2.0
    end = 1.0
    profile = MultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    profile.add_value(0.1, 1.03025)
    profile.add_value(0.2, 1.072)
    profile.add_value(0.4, 1.196)
    profile.add_value(0.5, 1.28125)
    profile.add_value(0.7, 1.50575)
    profile.add_value(0.8, 1.648)
    profile.add_value(0.9, 1.81225)

    assert math.isclose(profile.first_derivative_at(0.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(1.0), end - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.first_derivative_at(0.5), end - start, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_first_order_multi_point_profile():
    start = 1.0
    end = 2.0
    profile = MultiPointLinearProfile(start, end)

    assert math.isclose(profile.second_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -6 x^2 + 7x + 1
    start = 1.0
    end = 2.0
    profile = MultiPointLinearProfile(start, end)
    profile.add_value(0.5, 3.0)

    assert math.isclose(profile.second_derivative_at(0.0), -12.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), -12.0 - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), -12.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 2.0
    end = 1.0
    profile = MultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    assert math.isclose(profile.second_derivative_at(0.0), 1.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(1.0), 2.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.second_derivative_at(0.5), 1.75, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_second_derivative_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 2.0
    end = 1.0
    profile = MultiPointLinearProfile(start, end)
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
    profile = MultiPointLinearProfile(start, end)

    assert math.isclose(profile.third_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -6 x^2 + 7x + 1
    start = 1.0
    end = 2.0
    profile = MultiPointLinearProfile(start, end)
    profile.add_value(0.5, 3.0)

    assert math.isclose(profile.third_derivative_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 0.0 - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 2.0
    end = 1.0
    profile = MultiPointLinearProfile(start, end)
    profile.add_value(0.3, 1.12675)
    profile.add_value(0.6, 1.384)

    assert math.isclose(profile.third_derivative_at(0.0), 1.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(1.0), 1.5, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.third_derivative_at(0.5), 1.5, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_third_derivative_at_with_third_order_with_additional_points_multi_point_profile():
    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 2.0
    end = 1.0
    profile = MultiPointLinearProfile(start, end)
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
    profile = MultiPointLinearProfile(start, end)

    assert math.isclose(profile.value_at(0.0), start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), end, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), start + (end - start) / 2.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_second_order_multi_point_profile():

    # This gives: f(x) = -6 x^2 + 7x + 1
    start = 1.0
    end = 2.0
    profile = MultiPointLinearProfile(start, end)
    profile.add_value(0.5, 3.0)

    assert math.isclose(profile.value_at(0.0), 1.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), 2.0 - start, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), 3.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.25), 2.375, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.75), 2.875, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_third_order_multi_point_profile():

    # This gives: f(x) = 0.25 * x^3 + 0.5 * x^2 + 0.25 * x + 1
    start = 2.0
    end = 1.0
    profile = MultiPointLinearProfile(start, end)
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
    profile = MultiPointLinearProfile(start, end)
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

# CompoundProfile

def test_should_create_compound_profile_with_single_profile():
    profile = CompoundProfile()

    sub_profile = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 1.0, sub_profile)

    assert profile.value_at(0.0) == sub_profile.value_at(0.0)
    assert profile.value_at(0.5) == sub_profile.value_at(0.5)
    assert profile.value_at(1.0) == sub_profile.value_at(1.0)

def test_should_create_compound_profile_with_multiple_not_connected_profiles():
    profile = CompoundProfile()

    sub_profile_1 = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.2, sub_profile_1)

    sub_profile_2 = LinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.6, sub_profile_2)

    sub_profile_3 = LinearProfile(5.0, 6.0)
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
    profile = CompoundProfile()

    sub_profile_1 = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = LinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    sub_profile_3 = LinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    assert profile.value_at(0.0) == sub_profile_1.value_at(0.0)
    assert profile.value_at(0.2) == sub_profile_1.value_at(0.5)

    assert profile.value_at(0.4) == sub_profile_2.value_at(0.0)
    assert profile.value_at(0.55) == sub_profile_2.value_at(0.5)

    assert profile.value_at(0.7) == sub_profile_3.value_at(0.0)
    assert profile.value_at(0.85) == sub_profile_3.value_at(0.5)
    assert profile.value_at(1.0) == sub_profile_3.value_at(1.0)

def test_should_create_compound_profile_with_multiple_out_of_order_profiles():
    profile = CompoundProfile()

    sub_profile_3 = LinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    sub_profile_1 = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = LinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    assert profile.value_at(0.0) == sub_profile_1.value_at(0.0)
    assert profile.value_at(0.2) == sub_profile_1.value_at(0.5)

    assert profile.value_at(0.4) == sub_profile_2.value_at(0.0)
    assert profile.value_at(0.55) == sub_profile_2.value_at(0.5)

    assert profile.value_at(0.7) == sub_profile_3.value_at(0.0)
    assert profile.value_at(0.85) == sub_profile_3.value_at(0.5)
    assert profile.value_at(1.0) == sub_profile_3.value_at(1.0)

def test_should_throw_error_with_multiple_overlapping_sub_profiles_in_compound_profile():
    profile = CompoundProfile()

    sub_profile_1 = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.40, sub_profile_1)

    sub_profile_3 = LinearProfile(5.0, 6.0)
    profile.add_profile(0.60, 1.0, sub_profile_3)

    sub_profile_2 = LinearProfile(3.0, 4.0)

    with pytest.raises(InvalidTimeFractionException):
        profile.add_profile(0.35, 0.70, sub_profile_2)

def test_should_show_value_at_with_no_sub_profile_in_compound_profile():
    profile = CompoundProfile()

    assert math.isclose(profile.value_at(0.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(0.5), 0.0, rel_tol=1e-6, abs_tol=1e-15)
    assert math.isclose(profile.value_at(1.0), 0.0, rel_tol=1e-6, abs_tol=1e-15)

def test_should_show_value_at_with_single_sub_profile_that_covers_start_in_compound_profile():
    profile = CompoundProfile()

    sub_profile = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.2, sub_profile)

    assert profile.value_at(0.0) == sub_profile.value_at(0.0)
    assert profile.value_at(0.1) == sub_profile.value_at(0.5)
    assert profile.value_at(0.2) == sub_profile.value_at(1.0)

    assert profile.value_at(0.4) == sub_profile.value_at(1.0)
    assert profile.value_at(0.6) == sub_profile.value_at(1.0)
    assert profile.value_at(0.8) == sub_profile.value_at(1.0)
    assert profile.value_at(1.0) == sub_profile.value_at(1.0)

def test_should_show_value_at_with_single_sub_profile_that_covers_end_in_compound_profile():
    profile = CompoundProfile()

    sub_profile = LinearProfile(1.0, 2.0)
    profile.add_profile(0.8, 1.0, sub_profile)

    assert profile.value_at(0.0) == sub_profile.value_at(0.0)
    assert profile.value_at(0.2) == sub_profile.value_at(0.0)
    assert profile.value_at(0.4) == sub_profile.value_at(0.0)
    assert profile.value_at(0.6) == sub_profile.value_at(0.0)

    assert profile.value_at(0.8) == sub_profile.value_at(0.0)
    assert profile.value_at(0.9) == sub_profile.value_at(0.5)
    assert profile.value_at(1.0) == sub_profile.value_at(1.0)

def test_should_show_first_derivative_at_with_single_sub_profile_in_compound_profile():
    profile = CompoundProfile()

    sub_profile = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 1.0, sub_profile)

    assert profile.first_derivative_at(0.0) == sub_profile.first_derivative_at(0.0)
    assert profile.first_derivative_at(0.5) == sub_profile.first_derivative_at(0.5)
    assert profile.first_derivative_at(1.0) == sub_profile.first_derivative_at(1.0)

def test_should_show_first_derivative_at_with_multiple_sub_profiles_in_compound_profile():
    profile = CompoundProfile()

    sub_profile_1 = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = LinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    sub_profile_3 = LinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    assert profile.first_derivative_at(0.0) == sub_profile_1.first_derivative_at(0.0)
    assert profile.first_derivative_at(0.2) == sub_profile_1.first_derivative_at(0.5)

    assert profile.first_derivative_at(0.4) == sub_profile_2.first_derivative_at(0.0)
    assert profile.first_derivative_at(0.55) == sub_profile_2.first_derivative_at(0.5)

    assert profile.first_derivative_at(0.7) == sub_profile_3.first_derivative_at(0.0)
    assert profile.first_derivative_at(0.85) == sub_profile_3.first_derivative_at(0.5)
    assert profile.first_derivative_at(1.0) == sub_profile_3.first_derivative_at(1.0)

def test_should_show_second_derivative_at_with_single_sub_profile_in_compound_profile():
    profile = CompoundProfile()

    sub_profile = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 1.0, sub_profile)

    assert profile.second_derivative_at(0.0) == sub_profile.second_derivative_at(0.0)
    assert profile.second_derivative_at(0.5) == sub_profile.second_derivative_at(0.5)
    assert profile.second_derivative_at(1.0) == sub_profile.second_derivative_at(1.0)

def test_should_show_second_derivative_at_with_multiple_sub_profiles_in_compound_profile():
    profile = CompoundProfile()

    sub_profile_1 = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = LinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    sub_profile_3 = LinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    assert profile.second_derivative_at(0.0) == sub_profile_1.second_derivative_at(0.0)
    assert profile.second_derivative_at(0.2) == sub_profile_1.second_derivative_at(0.5)

    assert profile.second_derivative_at(0.4) == sub_profile_2.second_derivative_at(0.0)
    assert profile.second_derivative_at(0.55) == sub_profile_2.second_derivative_at(0.5)

    assert profile.second_derivative_at(0.7) == sub_profile_3.second_derivative_at(0.0)
    assert profile.second_derivative_at(0.85) == sub_profile_3.second_derivative_at(0.5)
    assert profile.second_derivative_at(1.0) == sub_profile_3.second_derivative_at(1.0)

def test_should_show_third_derivative_at_with_single_sub_profile_in_compound_profile():
    profile = CompoundProfile()

    sub_profile = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 1.0, sub_profile)

    assert profile.third_derivative_at(0.0) == sub_profile.third_derivative_at(0.0)
    assert profile.third_derivative_at(0.5) == sub_profile.third_derivative_at(0.5)
    assert profile.third_derivative_at(1.0) == sub_profile.third_derivative_at(1.0)

def test_should_show_third_derivative_at_with_multiple_sub_profiles_in_compound_profile():
    profile = CompoundProfile()

    sub_profile_1 = LinearProfile(1.0, 2.0)
    profile.add_profile(0.0, 0.4, sub_profile_1)

    sub_profile_2 = LinearProfile(3.0, 4.0)
    profile.add_profile(0.4, 0.70, sub_profile_2)

    sub_profile_3 = LinearProfile(5.0, 6.0)
    profile.add_profile(0.70, 1.0, sub_profile_3)

    assert profile.third_derivative_at(0.0) == sub_profile_1.third_derivative_at(0.0)
    assert profile.third_derivative_at(0.2) == sub_profile_1.third_derivative_at(0.5)

    assert profile.third_derivative_at(0.4) == sub_profile_2.third_derivative_at(0.0)
    assert profile.third_derivative_at(0.55) == sub_profile_2.third_derivative_at(0.5)

    assert profile.third_derivative_at(0.7) == sub_profile_3.third_derivative_at(0.0)
    assert profile.third_derivative_at(0.85) == sub_profile_3.third_derivative_at(0.5)
    assert profile.third_derivative_at(1.0) == sub_profile_3.third_derivative_at(1.0)

def test_should_show_inflection_points_with_single_profile():
    pytest.fail("not implemented yet")

def test_should_show_inflection_points_with_multiple_not_connected_profiles():
    pytest.fail("not implemented yet")

def test_should_show_inflection_points_with_multiple_connected_profiles():
    pytest.fail("not implemented yet")
