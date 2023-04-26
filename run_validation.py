from abc import ABC, abstractmethod
import argparse
from csv import DictReader
from distutils import util
from math import cos, isclose, isinf, pi, radians, sin, sqrt
import numpy as np
from os import makedirs, path
from pathlib import Path
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import plotly.offline as py
from random import random
import subprocess
from typing import List, Mapping, NamedTuple, Tuple
import yaml
from yaml.loader import SafeLoader

csv_time_column_name = "Time (s)"

body_variable_names: List[str] = [
    "x_position_body",
    "x_velocity_body",
    "x_acceleration_body",
    "x_jerk_body",
    "y_position_body",
    "y_velocity_body",
    "y_acceleration_body",
    "y_jerk_body",
    "orientation_body",
    "orientation_velocity_body",
    "orientation_acceleration_body",
    "orientation_jerk_body",
]

body_variable_mapping: Mapping[str, str] = {
    "x-body [wc] (m)": "x_position_body",
    "x-vel-body [bc] (m/s)": "x_velocity_body",
    "x-acc-body [bc] (m/s^2)": "x_acceleration_body",
    "x-jerk-body [bc] (m/s^3)": "x_jerk_body",
    "y-body [wc] (m)": "y_position_body",
    "y-vel-body [bc] (m/s)": "y_velocity_body",
    "y-acc-body [bc] (m/s^2)": "y_acceleration_body",
    "y-jerk-body [bc] (m/s^3)": "y_jerk_body",
    "z-rot-body [wc] (rad)": "orientation_body",
    "z-rotvel-body [bc] (rad/s)": "orientation_velocity_body",
    "z-rotacc-body [bc] (rad/s^2": "orientation_acceleration_body",
    "z-rotjerk-body [bc] (rad/s^3)": "orientation_jerk_body",
}

module_variable_names: List[str] = [
    "drive_velocity_module_0",
    "drive_acceleration_module_0",
    "drive_jerk_module_0",
    "steering_angle_position_module_0",
    "steering_angle_velocity_module_0",
    "steering_angle_acceleration_module_0",
    "steering_angle_jerk_module_0",

    "drive_velocity_module_1",
    "drive_acceleration_module_1",
    "drive_jerk_module_1",
    "steering_angle_position_module_1",
    "steering_angle_velocity_module_1",
    "steering_angle_acceleration_module_1",
    "steering_angle_jerk_module_1",

    "drive_velocity_module_2",
    "drive_acceleration_module_2",
    "drive_jerk_module_2",
    "steering_angle_position_module_2",
    "steering_angle_velocity_module_2",
    "steering_angle_acceleration_module_2",
    "steering_angle_jerk_module_2",

    "drive_velocity_module_3",
    "drive_acceleration_module_3",
    "drive_jerk_module_3",
    "steering_angle_position_module_3",
    "steering_angle_velocity_module_3",
    "steering_angle_acceleration_module_3",
    "steering_angle_jerk_module_3",
]

module_variable_mapping: Mapping[str, str] = {
    "x-vel-module-": "drive_velocity_module_{}",
    "x-acc-module-": "drive_acceleration_module_{}",
    "x-jerk-module-": "drive_jerk_module-{}",
    "z-rot-module-": "steering_angle_position_module_{}",
    "z-rotvel-module-": "steering_angle_velocity_module_{}",
    "z-rotacc-module-": "steering_angle_acceleration_module_{}",
    "z-rotjerk-module-": "steering_angle_jerk_module_{}",
}

class MotionProfile(ABC):

    @abstractmethod
    def movement_at_time(self, time: float) -> float:
        pass

class PolynomialMotionProfile(MotionProfile):

    def __init__(self, coefficients: List[float]):
        self.polynomial = np.poly1d(coefficients)

    def movement_at_time(self, time: float) -> float:
        return np.polyval(self.polynomial, time)

def motion_profile_from_yaml_element(elt) -> MotionProfile:
    type = elt["type"]
    if type == "polynomial":
        return PolynomialMotionProfile(elt["coeffients"])

class ValidationTrajectory(NamedTuple):
    start_time: float
    end_time: float
    body_coefficients: Mapping[str, MotionProfile]
    module_coeffients: Mapping[str, Mapping[str, MotionProfile]]

class ValidationPlan(NamedTuple):
    description: str
    name: str
    simulation_config_dir: str
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

            for validation_dir_name, validation_info_list in data_validations.items():
                for validation_info in validation_info_list:
                    print(validation_info)
                    include_simulation = validation_info["include"]
                    if not include_simulation:
                        continue

                    trajectories: List[ValidationTrajectory] = []
                    expected_results = validation_info["expected_results"]
                    for results_per_time_slot in expected_results:

                        body_results_per_time = results_per_time_slot["body"]
                        body_coefficients: Mapping[str, MotionProfile] = {}
                        for name in body_variable_names:
                            motion_profile = motion_profile_from_yaml_element(body_results_per_time[name])
                            body_coefficients[name] = motion_profile

                        modules_results_per_time = results_per_time_slot["modules"]
                        module_coefficients: Mapping[str, Mapping[str, MotionProfile]] = {}
                        for module_results in modules_results_per_time:
                            module_name = module_results["name"]
                            module_validation: Mapping[str, List[float]] = {}
                            for name in module_variable_names:
                                module_validation[name] = module_results[name],

                            module_coefficients[module_name] = module_validation

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
                        simulation_config_dir=validation_dir_name,
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

    print(args)
    return vars(args)

def read_simulation_output(simulation_name: str, validation_directory: str) -> Mapping[str, Tuple[List[float], List[float]]]:
    calculated_profiles: Mapping[str, Tuple[List[float], List[float]]] = {}
    csv_file_path = path.join(validation_directory, "{}.csv".format(simulation_name))
    with open(csv_file_path, mode='r') as csv_file:
        csv_reader = DictReader(csv_file)
        line_count = -1
        for row in csv_reader:
            line_count += 1
            if line_count == 0:
                continue

            for csv_variable_name in row:
                if csv_variable_name == csv_time_column_name:
                    continue

                validation_name: str = ""
                if csv_variable_name in body_variable_mapping:
                    validation_name = body_variable_mapping[csv_variable_name]

                # Modules are numbered ... Suck
                for key in module_variable_mapping:
                    if csv_variable_name.startswith(key):
                        index1 = csv_variable_name.find(key) + len(key)
                        index2 = csv_variable_name.find(" ", index1)

                        module_index_str = csv_variable_name[ index1 : index2 ]

                        validation_name = module_variable_mapping[key].format(module_index_str)

                if validation_name == "":
                    continue

                if not validation_name in calculated_profiles.keys():
                    calculated_profiles[validation_name] = ( [], [] )

                pair = calculated_profiles[validation_name]
                pair[0].append(row[csv_time_column_name])
                pair[1].append(row[csv_variable_name])

    return calculated_profiles

def run_validations(arg_dict: Mapping[str, any]):
    input_files: List[str] = arg_dict["file"]
    output_directory: str = path.abspath(arg_dict["output"])
    print("Running validation ...")
    print("Validating simulation code using the following files:")
    for input_file in input_files:
        print("    {}".format(input_file))

    print("Outputting to {}".format(output_directory))

    motions = get_validations(input_files)
    for validation_set in motions:
        validation_directory = path.join(output_directory, validation_set.simulation_config_dir, validation_set.name)
        validation_run_simulation(validation_directory, validation_set)

def validation_run_simulation(validation_directory: str, plan: ValidationPlan):
    simulation_file = plan.input_file_path

    # Run the simulation
    command_str = "python run_trajectory_simulation.py --no-graphs --output {} --file {}".format(validation_directory, simulation_file)
    process_result = subprocess.run(command_str, shell=True)
    if process_result.returncode != 0:
        # Simulation failed for some reason
        raise "stuffed"

    calculated_profiles = read_simulation_output(plan.name, validation_directory)

    for approximation in plan.approximations:
        # approximation == ValidationTrajectory
        #   approximation.body_coefficients == Mapping[str, Profile] -> str == name of variable
        #   approximation.module_coeffients == Mapping[str, Mapping[str, Profile]] str_1 == name of module, str_2 == name of variable

        # Body compare

        pass


    for key, profile in calculated_profiles.items():
         # key == name of the profile

         if key in body_variable_mapping:
            plan.approximations
            validation_name = body_variable_mapping[csv_variable_name]

        for key in module_variable_mapping:
            if csv_variable_name.startswith(key):

        # IF the key has _body then look at the body profiles
        # if the key has _module_X then look at the module profiles

        # Get the right approximation
        pass

    #   assemble the polynomial
    #   get the values
    #   calculate the error between the provided polynomial and the values
    #   create a graph
    #   record the errors somewhere

    pass

def main(args=None):
    arg_dict = read_arguments()

    run_validations(arg_dict)

if __name__ == '__main__':
    main()
