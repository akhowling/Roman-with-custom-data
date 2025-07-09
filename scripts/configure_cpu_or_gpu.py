
import argparse
import os
from pathlib import Path
import glob

def change_yaml_tag(file_pattern, tag, value):
    """
    Change the specified tag in the YAML file to the given value.
    """
    matching_files = glob.glob(file_pattern, recursive=True)
    for file in matching_files:
        with open(file, 'r') as f:
            lines = f.readlines()
        
        # replace the tag value
        for i, line in enumerate(lines):
            if line.strip().startswith(f"{tag}:"):
                lines[i] = f"{tag}: {value}\n"
        
        # print(lines)
        with open(file, 'w') as f:
            f.writelines(lines)
    

if __name__ == '__main__':
    info_str = """
    This script is used to configure the ROMAN repo for using either GPU or only CPU.
    Normally, ROMAN uses GPU to extract segments from an image and to create 
    semantic embeddings for the segments. ROMAN can still function on CPU, but a 
    few things need to be configured. This script handles reconfiguring the repo for 
    use with CPU. When running with CPU, ROMAN will likely run much slower, and performance
    will be reduced.
    """
    parser = argparse.ArgumentParser(description=info_str)
    parser.add_argument('-d', '--device', type=str, default='cpu', choices=['cpu', 'gpu'],
                        help='Specify the device to use: "cpu" for CPU or "gpu" for GPU. Default is "cpu".')
    args = parser.parse_args()

    script_path = os.path.realpath(__file__)
    script_dir = os.path.dirname(script_path)
    roman_ros_repo_dir = os.path.dirname(script_dir)

    if args.device == 'cpu':
        change_yaml_tag(f"{roman_ros_repo_dir}/**/fastsam.yaml", "device", "cpu")
        change_yaml_tag(f"{roman_ros_repo_dir}/**/fastsam.yaml", "semantics", "none")
        change_yaml_tag(f"{roman_ros_repo_dir}/**/submap_align.yaml", "method", "roman_no_semantics")
        change_yaml_tag(f"{roman_ros_repo_dir}/**/roman_loop_closure.yaml", "method", "roman_no_semantics")

        print("Repo now configured for CPU usage. ROMAN will run slower, and performance will be reduced.")
        print("To apply changes, run colcon build in teh ROS2 workspace.")
        print()
        print("To revert to GPU usage, run this script again with the --device gpu flag.")

    elif args.device == 'gpu':
        change_yaml_tag(f"{roman_ros_repo_dir}/**/fastsam.yaml", "device", "'cuda'")
        change_yaml_tag(f"{roman_ros_repo_dir}/**/fastsam.yaml", "semantics", "'dino'")
        change_yaml_tag(f"{roman_ros_repo_dir}/**/submap_align.yaml", "method", "roman")
        change_yaml_tag(f"{roman_ros_repo_dir}/**/roman_loop_closure.yaml", "method", "roman")

        print("Repo now configured for GPU usage.")
        print("To apply changes, run colcon build in the ROS2 workspace.")
        print()
        print("To configure for running with CPU, run this script again with the --device cpu flag.")