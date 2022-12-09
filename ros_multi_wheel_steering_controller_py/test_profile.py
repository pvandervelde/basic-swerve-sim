import math
import pytest
from typing import Mapping, List, Tuple

# locals
from .profile import CompoundProfile, InvalidTimeFractionException, LinearProfile

# LinearProfile

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
    pytest.fail("not implemented yet")

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
    pytest.fail("not implemented yet")

def test_should_show_first_derivative_at_with_multiple_sub_profiles_in_compound_profile():
    pytest.fail("not implemented yet")

def test_should_show_second_derivative_at_with_single_sub_profile_in_compound_profile():
    pytest.fail("not implemented yet")

def test_should_show_second_derivative_at_with_multiple_sub_profiles_in_compound_profile():
    pytest.fail("not implemented yet")
