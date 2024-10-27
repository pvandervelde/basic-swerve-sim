import argparse
from pathlib import Path
from typing import Mapping

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def read_arguments() -> Mapping[str, any]:
    parser = argparse.ArgumentParser(
        description="Read a rosbag and extract the desired motions to be executed.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        "-d",
        "--directory",
        action="store",
        required=True,
        type=str,
        help="The path for the ros bag directory which contains the desired motions to be executed.",
    )

    args = parser.parse_args()

    return vars(args)


def main(args=None):
    arg_dict = read_arguments()
    input_directory: str = arg_dict["directory"]

    bagpath = Path(input_directory)

    # Create a type store to use if the bag has no message definitions.
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Create reader instance and open for reading.
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == "/cmd_vel"]
        start_time_in_nano_seconds: int = reader.start_time
        end_time_in_nano_seconds: int = reader.end_time
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            time_since_start_in_seconds: float = (
                timestamp - start_time_in_nano_seconds
            ) / 1e9

            msg = reader.deserialize(rawdata, connection.msgtype)
            print(f"connection.id: {connection.id}")
            print(f"connection.ext: {connection.ext}")
            print(f"connection.msgtype: {connection.msgtype}")
            print(f"connection.topic: {connection.topic}")
            print(f"connection.owner: {connection.owner}")
            print(f"connection.index: {connection.index}")

            # print(f"Header id: {msg.header.frame_id}")
            # print(
            #    f"{time_since_start_in_seconds} Twist - linear: [{msg.linear.x}, {msg.linear.y}, 0.0] angular: [0.0, 0.0, {msg.angular.z}]"
            # )


if __name__ == "__main__":
    main()
