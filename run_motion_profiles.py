import argparse
from os import makedirs, path

from typing import List, Mapping, Tuple
from sim_output.animate_motion_profile import plot_profile
from swerve_controller.profile import SingleVariableLinearProfile, SingleVariableSCurveProfile, SingleVariableTrapezoidalProfile, TransientVariableProfile

def get_linear_motion_profile(start: float, end: float) -> TransientVariableProfile:
    return SingleVariableLinearProfile(start, end)

def get_scurve_profile(start: float, end: float) -> TransientVariableProfile:
    return SingleVariableSCurveProfile(start, end)

def get_trapezoidal_profile(start: float, end: float) -> TransientVariableProfile:
    return SingleVariableTrapezoidalProfile(start, end)

def read_arguments() -> Mapping[str, any]:
    parser = argparse.ArgumentParser(
        description="Simulate motion profile",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        "-o",
        "--output",
        action="store",
        required=True,
        type=str,
        help="The directory path for the output files")

    args = parser.parse_args()

    return vars(args)

def simulation_run_motion_profiles(arg_dict: Mapping[str, any]):
    output_directory: str = arg_dict["output"]
    print("Running motion profiles")

    print("Outputting to {}".format(output_directory))

    start = 0.0
    end = 10.0

    profiles: List[Tuple[str, TransientVariableProfile]] = [
        ('linear', get_linear_motion_profile(start, end)),
        ('trapezoidal', get_trapezoidal_profile(start, end)),
        ('s-curve', get_scurve_profile(start, end)),
    ]

    motion_directory = path.join(output_directory, 'motion_profiles')
    if not path.isdir(motion_directory):
        print("Output directory {} does not exist. Creating directory ...".format(motion_directory))
        makedirs(motion_directory)

    simulation_rate_in_hz = 100
    current_sim_time_in_seconds = 0.0

    points_in_time: List[float] = [ ]
    positions: List[List[float]] = [ [] for pair in profiles ]
    velocities: List[List[float]] = [ [] for pair in profiles ]
    accelerations: List[List[float]] = [ [] for pair in profiles ]
    jerks: List[List[float]] = [ [] for pair in profiles ]

    # Assume the movement takes 1.0 seconds
    profile_motion_time = 1.0
    step_count = int(profile_motion_time * simulation_rate_in_hz)
    time_step_in_seconds =  1.0 / float(simulation_rate_in_hz)

    for time_index in range(1, step_count + 2):
        print("Processing step at {} ...".format(current_sim_time_in_seconds))
        points_in_time.append(current_sim_time_in_seconds)

        for index, pair in enumerate(profiles):
            profile = pair[1]
            positions[index].append(profile.value_at(current_sim_time_in_seconds / profile_motion_time))
            velocities[index].append(profile.first_derivative_at(current_sim_time_in_seconds / profile_motion_time))
            accelerations[index].append(profile.second_derivative_at(current_sim_time_in_seconds / profile_motion_time))
            jerks[index].append(profile.third_derivative_at(current_sim_time_in_seconds / profile_motion_time))

        current_sim_time_in_seconds += time_step_in_seconds

    # Now draw all the graphs
    plot_file_path = path.join(output_directory, "motion_profile")
    plot_profile(
        [x[0] for x in profiles],
        points_in_time,
        positions,
        velocities,
        accelerations,
        jerks,
        plot_file_path)

def main(args=None):
    arg_dict = read_arguments()

    simulation_run_motion_profiles(arg_dict)

if __name__ == '__main__':
    main()