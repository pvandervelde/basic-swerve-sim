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

class VerificationPlan(NamedTuple):
    description: str
    name: str
    simulation_config_dir: str
    input_file_path: str

def get_verifications(input_files: List[str]) -> List[VerificationPlan]:
    result: List[VerificationPlan] = []

    for input_file in input_files:
        relative = Path(input_file)

        with open(relative.absolute()) as f:
            print("Reading {} ...".format(f.name))
            data = yaml.load(f, Loader=SafeLoader)
            verification_base_dir: str = data["verification"]["verification_base_directory"]
            data_verifications = data["verification"]["verifications"]

            for verification_dir_name, verification_info_list in data_verifications.items():
                for verification_info in verification_info_list:
                    print(verification_info)
                    include_simulation = verification_info["include"]
                    if not include_simulation:
                        continue

                    plan = VerificationPlan(
                        description=verification_info["description"],
                        name=verification_info["name"],
                        simulation_config_dir=verification_dir_name,
                        input_file_path=path.abspath(path.join(verification_base_dir, verification_dir_name, verification_info["simulation_file"])),
                    )

                    result.append(plan)

    return result

def read_arguments() -> Mapping[str, any]:
    parser = argparse.ArgumentParser(
        description="Run verifications on the swerve drive simulation code.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument(
        "-f",
        "--file",
        action="append",
        required=True,
        type=str,
        help="The file path for the input file which contains the desired verifications to be executed. Can be provided multiple times.")

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

def run_verifications(arg_dict: Mapping[str, any]):
    input_files: List[str] = arg_dict["file"]
    output_directory: str = path.abspath(arg_dict["output"])
    print("Running verification ...")
    start = datetime.now()

    print("Validating simulation code using the following files:")
    for input_file in input_files:
        print("    {}".format(input_file))

    print("Outputting to {}".format(output_directory))

    motions = get_verifications(input_files)
    for verification_set in motions:
        verification_directory = path.join(output_directory, verification_set.simulation_config_dir)
        verification_run_simulation(verification_directory, verification_set)

    end = datetime.now()
    difference = (end - start).total_seconds()

    print("Total processing time: {} seconds".format(difference))

def verification_run_simulation(controller_name: str, verification_directory: str, plan: VerificationPlan):
    simulation_file = plan.input_file_path

    # Run the simulation
    command_str = "python run_trajectory_simulation.py --no-graphs --output {} --file {}".format(verification_directory, simulation_file)
    process_result = subprocess.run(command_str, shell=True)
    if process_result.returncode != 0:
        # Simulation failed for some reason
        raise "stuffed"

def main(args=None):
    arg_dict = read_arguments()

    run_verifications(arg_dict)

if __name__ == '__main__':
    main()
