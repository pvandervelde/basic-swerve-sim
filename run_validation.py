from abc import ABC, abstractmethod
import argparse
from csv import DictReader
from datetime import datetime
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

class ValidationPlan(NamedTuple):
    description: str
    name: str
    simulation_config_dir: str
    input_file_path: str

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

                    plan = ValidationPlan(
                        description=validation_info["description"],
                        name=validation_info["name"],
                        simulation_config_dir=validation_dir_name,
                        input_file_path=path.abspath(path.join(validation_base_dir, validation_dir_name, validation_info["simulation_file"])),
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

def run_validations(arg_dict: Mapping[str, any]):
    input_files: List[str] = arg_dict["file"]
    output_directory: str = path.abspath(arg_dict["output"])
    print("Running validation ...")
    start = datetime.now()

    print("Validating simulation code using the following files:")
    for input_file in input_files:
        print("    {}".format(input_file))

    print("Outputting to {}".format(output_directory))

    motions = get_validations(input_files)
    for validation_set in motions:
        validation_directory = path.join(output_directory, validation_set.simulation_config_dir)
        validation_run_simulation(validation_directory, validation_set)

    end = datetime.now()
    difference = (end - start).total_seconds()

    print("Total processing time: {} seconds".format(difference))

def validation_run_simulation(validation_directory: str, plan: ValidationPlan):
    simulation_file = plan.input_file_path

    # Run the simulation
    command_str = "python run_trajectory_simulation.py --no-graphs --output {} --file {}".format(validation_directory, simulation_file)
    process_result = subprocess.run(command_str, shell=True)
    if process_result.returncode != 0:
        # Simulation failed for some reason
        raise "stuffed"

def main(args=None):
    arg_dict = read_arguments()

    run_validations(arg_dict)

if __name__ == '__main__':
    main()
