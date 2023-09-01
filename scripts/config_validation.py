#!/usr/bin/env python3
import yaml
import sys

def extract_annotations(file_path):
    annotations = {}
    key_path_stack = []  # Stack to maintain the current path
    indentation_to_depth = []

    with open(file_path, 'r') as f:
        for line in f:
            stripped_line = line.lstrip()
            if not stripped_line:
                continue  # Skip empty lines

            current_indentation = len(line) - len(stripped_line)

            if current_indentation > len(indentation_to_depth) - 1:
                # We've gone one level deeper
                while current_indentation > len(indentation_to_depth) - 1:
                    indentation_to_depth.append(len(key_path_stack))
            else:
                # We've gone back up, so we set our stack to the known depth
                key_path_stack = key_path_stack[:indentation_to_depth[current_indentation]]

            current_key = stripped_line.split(':')[0].strip()
            if "#optional" in stripped_line:
                annotations[tuple(key_path_stack + [current_key])] = 'optional'
            else:
                annotations[tuple(key_path_stack + [current_key])] = 'required'

            # Update the key path for the next iteration
            key_path_stack.append(current_key)

    return annotations


def validate_config_recursive(user_data, default_data, annotations, key_path=[]):
    all_valid = True

    for key, default_value in default_data.items():
        current_path = key_path + [key]
        tuple_path = tuple(current_path)
        if tuple_path in annotations:
            status = annotations[tuple_path]

            # Check if key exists in user data for required keys
            if status == 'required' and key not in user_data:
                print(f"Key {'.'.join(current_path)} is required but missing from the user YAML")
                all_valid = False

            # If the key is optional and is missing in user data, then skip checking its children
            elif status == 'optional' and key not in user_data:
                continue

            # Check type if key exists in both
            elif key in user_data:
                if type(user_data[key]) != type(default_value):
                    print(f"Key {'.'.join(current_path)} has a different type in user YAML. Expected type: {type(default_value)}, found type: {type(user_data[key])}")
                    all_valid = False

                # Recursive validation for nested dictionaries
                elif isinstance(default_value, dict):
                    if not validate_config_recursive(user_data[key], default_value, annotations, current_path):
                        all_valid = False
        # If a key from default config is not in the annotations, it's assumed required.
        elif key not in user_data:
            print(f"Key {'.'.join(current_path)} is required since not #optional but missing from the user YAML")
            all_valid = False

    return all_valid

def validate_config(user_file, default_file):
    with open(user_file, 'r') as f:
        user_data = yaml.safe_load(f)

    with open(default_file, 'r') as f:
        default_data = yaml.safe_load(f)

    annotations = extract_annotations(default_file)

    return validate_config_recursive(user_data, default_data, annotations)

if __name__ == "__main__":
    if not validate_config(sys.argv[1], sys.argv[2]):
        sys.exit(1)
