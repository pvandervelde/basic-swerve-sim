import argparse
from math import cos, isclose, isinf, pi, radians, sin, sqrt
from os import makedirs, path
from pathlib import Path
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import plotly.offline as py
from random import random
from typing import List, Mapping, NamedTuple, Tuple
import yaml
from yaml.loader import SafeLoader

class BodyValidationTrajectory(NamedTuple):
    x_position : List[float]
    x_velocity : List[float]
    x_acceleration: List[float]
    x_jerk: List[float]

    y_position : List[float]
    y_velocity : List[float]
    y_acceleration: List[float]
    y_jerk: List[float]

    orientation_position : List[float]
    orientation_velocity : List[float]
    orientation_acceleration: List[float]
    orientation_jerk: List[float]

class ModuleValidationTrajectory(NamedTuple):
    drive_velocity: List[float]
    drive_acceleration: List[float]
    drive_jerk: List[float]

    steering_angle_position: List[float]
    steering_angle_velocity: List[float]
    steering_angle_acceleration: List[float]
    steering_angle_jerk: List[float]

class ValidationTrajectory(NamedTuple):
    start_time: float
    end_time: float
    body_coefficients: BodyValidationTrajectory
    module_coeffients: Mapping[str, ModuleValidationTrajectory]

class ValidationPlan(NamedTuple):
    description: str
    name: str
    output_directory: str
    input_file_path: str
    approximations: List[ValidationTrajectory]

def get_validations(input_files: List[str]) -> List[ValidationPlan]:
    result: List[ValidationPlan] = []

    for input_file in input_files:
        relative = Path(input_file)

        with open(relative.absolute()) as f:
            print("Reading {} ...".format(f.name))
            data = yaml.load(f, Loader=SafeLoader)
            validation_base_dir: str = data["validation"]["validation_base_directory"]
            data_validations = data["validation"]["validations"]

            for validation_dir_name, validation_info in data_validations.items():
                if not validation_info["include"]:
                    continue

                trajectories: List[ValidationTrajectory] = []
                expected_results = validation_info["expected_results"]
                for results_per_time_slot in expected_results:

                    body_results_per_time = results_per_time_slot["body"]
                    body_coefficients = BodyValidationTrajectory(
                        x_position=body_results_per_time["x_position_coefficients"],
                        x_velocity=body_results_per_time["x_velocity_coefficients"],
                        x_acceleration=body_results_per_time["x_acceleration_coefficients"],
                        x_jerk=body_results_per_time["x_jerk_coefficients"],
                        y_position=body_results_per_time["y_position_coefficients"],
                        y_velocity=body_results_per_time["y_velocity_coefficients"],
                        y_acceleration=body_results_per_time["y_acceleration_coefficients"],
                        y_jerk=body_results_per_time["y_jerk_coefficients"],
                        orientation_position=body_results_per_time["orientation_coefficients"],
                        orientation_velocity=body_results_per_time["orientation_velocity_coefficients"],
                        orientation_acceleration=body_results_per_time["orientation_acceleration_coefficients"],
                        orientation_jerk=body_results_per_time["orientation_jerk_coefficients"],
                    )

                    modules_results_per_time = results_per_time_slot["modules"]
                    module_coefficients: Mapping[str, ModuleValidationTrajectory] = {}
                    for module_results in modules_results_per_time:
                        name = module_results["name"]
                        module_coefficients=ModuleValidationTrajectory(
                            drive_velocity=module_results["drive_velocity_coefficients"],
                            drive_acceleration=module_results["drive_acceleration_coefficients"],
                            drive_jerk=module_results["drive_jerk_coefficients"],
                            steering_angle_position=module_results["steering_angle_position_coefficients"],
                            steering_angle_velocity=module_results["steering_angle_velocity_coefficients"],
                            steering_angle_acceleration=module_results["steering_angle_acceleration_coefficients"],
                            steering_angle_jerk=module_results["steering_angle_jerk_coefficients"],
                        )

                        module_coefficients[name] = module_coefficients

                    trajectory = ValidationTrajectory(
                        start_time=results_per_time_slot["time_start"],
                        end_time=results_per_time_slot["time_end"],
                        body_coefficients=body_coefficients,
                        module_coeffients=module_coefficients,
                    )

                    trajectories.append(trajectory)

                plan = ValidationPlan(
                    description=validation_info["description"],
                    name=validation_info["name"],
                    output_directory=validation_info["output_directory"],
                    input_file_path=path.abspath(path.join(validation_base_dir, validation_dir_name, validation_info["simulation_file"])),
                    approximations=trajectories
                )

                result.append(plan)

    return result

def read_arguments() -> Mapping[str, any]:
    parser = argparse.ArgumentParser(
        description="Run validations on the swerve drive simulation code.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        "-f",
        "--file",
        action="append",
        required=True,
        type=str,
        help="The file path for the input file which contains the desired validations to be executed. Can be provided multiple times.")

    parser.add_argument(
        "-o",
        "--output",
        action="store",
        required=True,
        type=str,
        help="The directory path for the output files")
    args = parser.parse_args()
    return vars(args)

def run_validations(arg_dict: Mapping[str, any]):
    input_files: List[str] = arg_dict["file"]
    output_directory: str = arg_dict["output"]
    print("Running validation ...")
    print("Validating simulation code using the following files:")
    for input_file in input_files:
        print("    {}".format(input_file))

    print("Outputting to {}".format(output_directory))

    motions = get_validations(input_files)
    for validation_set in motions:
        validation_directory = path.join(output_directory, validation_set.name)
        validation_run_simulation(validation_directory, validation_set)

def validation_run_simulation(validation_directory: str, plan: ValidationPlan):
    # Get the validation config

    # for each validation option
    #  get the simulation config
    #  run the simulation
    #  read the results
    #  match results to expected result
    pass

def main(args=None):
    arg_dict = read_arguments()

    run_validations(arg_dict)