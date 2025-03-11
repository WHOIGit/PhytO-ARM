#!/usr/bin/env python3
from dataclasses import dataclass, field as dc_field
import json
import select
import socket
import time
from typing import Any, Dict, List, Tuple

import rospy
from std_msgs.msg import Float64, String, Int64, Bool, Float64MultiArray, Int64MultiArray, Int8MultiArray
from ds_core_msgs.msg import RawData


# Type mapping for ROS message types
ROS_TYPE_MAP = {
    "float": Float64,
    "str": String,
    "int": Int64,
    "bool": Bool,
    "float[]": Float64MultiArray,
    "int[]": Int64MultiArray,
    "bool[]": Int8MultiArray,
}

VALID_CONNECTION_TYPES = ["udp", "tcp"]
VALID_PARSING_STRATEGIES = ["json_dict", "json_array", "raw", "delimited"]

def validate_config(config: dict) -> Tuple[bool, str]:
    """Validate complete network_data_capture configuration

    Args:
        config: Dict containing complete node configuration

    Returns:
        tuple[bool, str]: (is_valid, error_message)
    """
    if "topics" not in config:
        return False, "network_data_capture config must contain 'topics' dictionary"

    if not isinstance(config["topics"], dict):
        return False, "network_data_capture topics are incorrectly, see example.yaml"

    for topic_name, topic_config in config["topics"].items():
        is_valid, error = validate_topic_config(topic_name, topic_config)
        if not is_valid:
            return False, f"Invalid topic '{topic_name}': {error}"

    return True, ""

def validate_topic_config(topic_name: str, topic_config: dict) -> Tuple[bool, str]:
    """Validate single topic configuration including connection, parsing strategy, and subtopics

    Args:
        topic_name: Name of the topic
        topic_config: Dict containing topic configuration

    Returns:
        tuple[bool, str]: (is_valid, error_message)
    """
    required_fields = ["connection_type", "port", "parsing_strategy"]
    for field in required_fields:
        if field not in topic_config:
            return False, f"Missing required field '{field}'"

    if topic_config["connection_type"] not in VALID_CONNECTION_TYPES:
        return False, f"Invalid connection_type. Must be one of {VALID_CONNECTION_TYPES}"

    if not isinstance(topic_config["port"], int):
        return False, "Port must be an integer"

    if topic_config["parsing_strategy"] not in VALID_PARSING_STRATEGIES:
        return False, f"Invalid parsing_strategy. Must be one of {VALID_PARSING_STRATEGIES}"

    # Validate delimiter if delimited strategy
    if topic_config["parsing_strategy"] == "delimited":
        if "delimiter" not in topic_config:
            return False, "Delimiter required for delimited parsing strategy"
        if not isinstance(topic_config["delimiter"], str):
            return False, "Delimiter must be a string"

    # Validate subtopics if not raw
    if topic_config["parsing_strategy"] != "raw":
        if "subtopics" not in topic_config:
            return False, f"Subtopics required for {topic_config['parsing_strategy']} parsing strategy"
        if not isinstance(topic_config["subtopics"], dict):
            return False, "Subtopics must be a dictionary"

        for subtopic_name, _ in topic_config["subtopics"].items():
            is_valid, error = validate_subtopic_config(topic_config, subtopic_name)
            if not is_valid:
                return False, f"Invalid subtopic '{subtopic_name}': {error}"

    return True, ""

def validate_subtopic_config(topic_config: dict, subtopic_name: str) -> Tuple[bool, str]:
    """Validate single subtopic configuration including field_id and type

    Args:
        subtopic_name: Name of the subtopic
        subtopic_config: Dict containing subtopic configuration

    Returns:
        tuple[bool, str]: (is_valid, error_message)
    """
    subtopic_config = topic_config["subtopics"][subtopic_name]
    required_fields = ["field_id", "type"]
    for field in required_fields:
        if field not in subtopic_config:
            return False, f"Missing required field '{field}'"

    if topic_config["parsing_strategy"] == "delimited":
        if  subtopic_config["type"].endswith('[]'):
            return False, "Arrays are not a supported field type for delimited string parsing"

    if not isinstance(subtopic_config["field_id"], (int, str)):
        return False, "field_id must be an integer or string"

    if not isinstance(subtopic_config["type"], str):
        return False, "type must be a string"

    if subtopic_config["type"] not in ROS_TYPE_MAP:
        return False, f"Invalid type. Must be one of {list(ROS_TYPE_MAP.keys())}"

    return True, ""

def convert_to_ros_msg(value: Any, type_hint: str) -> Tuple[Any, str]:
    """Convert value to appropriate ROS message type based on config type_hint

    Args:
        value: The value to convert
        type_hint: String from config like "float", "str", "int", "bool", etc.

    Returns:
        tuple[Any, str]: (ros_message, error_message)
    """
    try:
        msg_type = ROS_TYPE_MAP[type_hint]

        # Try to manually validate the type since ROS messages don't always raise errors
        if type_hint == "float":
            if not isinstance(value, (int, float)):
                return None, f"Expected float for type {type_hint}, got {type(value)}"
        elif type_hint == "int":
            if not isinstance(value, int):
                return None, f"Expected int for type {type_hint}, got {type(value)}"
        elif type_hint == "bool":
            if not isinstance(value, bool):
                return None, f"Expected bool for type {type_hint}, got {type(value)}"
        elif type_hint == "str":
            if not isinstance(value, str):
                return None, f"Expected str for type {type_hint}, got {type(value)}"
        elif type_hint.endswith("[]") and not isinstance(value, (list, tuple)):
            return None, f"Expected list for type {type_hint}, got {type(value)}"

        # Create appropriate array message
        if type_hint == "bool[]":
            # Convert boolean array to int8 array
            msg = msg_type(data=[int(x) for x in value])
        else:
            msg = msg_type(data=value)

        return msg, ""

    except (ValueError, TypeError) as e:
        return None, f"Error converting value to {type_hint}: {str(e)}"

def create_socket(conn_type: str, port: int) -> Tuple[socket.socket, str]:
    """Create and bind either UDP or TCP socket

    Args:
        conn_type: Either "udp" or "tcp"
        port: Port number to bind to

    Returns:
        tuple[socket.socket, str]: (socket, error_message)
    """
    try:
        if conn_type == "udp":
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        else:  # tcp
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        sock.bind(("0.0.0.0", port))

        if conn_type == "tcp":
            sock.listen(1)

        return sock, ""

    except socket.error as e:
        return None, f"Socket error: {str(e)}"

def read_available_data(sockets: Dict[str, socket.socket]) -> Dict[str, Tuple[bytes, str]]:
    """Use select() to read available data from all sockets

    Args:
        sockets: Dict of topic_name -> socket

    Returns:
        dict[str, tuple[bytes, str]]: Dict of topic_name -> (data, error_message)
    """
    result = {}
    try:
        readable, _, _ = select.select(list(sockets.values()), [], [], 0.1)

        for sock in readable:

            # Find topic name for this socket
            topic_name = next(name for name, s in sockets.items() if s == sock)

            try:
                data, _ = sock.recvfrom(65536) # Read up to 64KB at a time
                result[topic_name] = (data, "")
            except socket.error as e:
                result[topic_name] = (b"", f"Error reading from socket: {str(e)}")

    except select.error as e:

        # Return error for all sockets
        return {name: (b"", f"Select error: {str(e)}") for name in sockets}

    return result

def append_to_buffer(buffer: bytes, new_data: bytes, max_size: int = 1048576) -> Tuple[bytes, str]:
    """Append new data to buffer, with size limit

    Args:
        buffer: Existing buffer
        new_data: New data to append
        max_size: Maximum allowed buffer size in bytes

    Returns:
        tuple[bytes, str]: (updated_buffer, error_message)
    """
    if len(buffer) + len(new_data) > max_size:
        return buffer, f"Buffer overflow: combined size {len(buffer) + len(new_data)} exceeds limit {max_size}"

    return buffer + new_data, ""

def extract_messages(buffer: bytes, strategy: str, config: dict) -> Tuple[List[bytes], bytes, str]:
    """Extract complete messages from buffer based on strategy

    Args:
        buffer: Current buffer containing received data
        strategy: Parsing strategy ("json_dict", "json_array", "raw", "delimited")
        config: Configuration dict containing strategy-specific settings

    Returns:
        tuple[list[bytes], bytes, str]: (complete_messages, remaining_buffer, error_message)
    """
    messages = []

    if strategy == "raw":
        while b'\n' in buffer:
            idx = buffer.find(b'\n')
            messages.append(buffer[:idx])
            buffer = buffer[idx + 1:]

    elif strategy == "delimited":
        delimiter = config["delimiter"].encode('utf-8')

        # Split on newlines first, then process delimited fields
        while b'\n' in buffer:
            idx = buffer.find(b'\n')
            message = buffer[:idx]
            if delimiter in message:  # Only add if it contains the delimiter
                messages.append(message)
            buffer = buffer[idx + 1:]

    elif strategy in ["json_dict", "json_array"]:

        # Process complete JSON objects
        decoder = json.JSONDecoder()
        while buffer:
            try:
                if not buffer.strip():  # Skip empty buffers
                    break

                # Try to decode JSON at current position
                try:
                    _, idx = decoder.raw_decode(buffer.decode('utf-8'))

                    # Extract the exact bytes that made up this message
                    msg_bytes = buffer[:idx].strip()
                    messages.append(msg_bytes)
                    buffer = buffer[idx:].lstrip()
                except json.JSONDecodeError:

                    # If we can't decode JSON, we need more data
                    break

            except UnicodeDecodeError as e:
                return [], buffer, f"Invalid UTF-8 in buffer: {str(e)}"

    else:
        return [], buffer, f"Unknown parsing strategy: {strategy}"

    return messages, buffer, ""

def parse_message(message: bytes, strategy: str, config: dict) -> Tuple[dict, str]:
    """Parse a single message according to strategy

    Args:
        message: Raw message bytes
        strategy: Parsing strategy ("json_dict", "json_array", "raw", "delimited")
        config: Configuration dict containing strategy-specific settings

    Returns:
        tuple[dict, str]: (parsed_fields, error_message)
        parsed_fields will be a dict mapping subtopic names to values
    """
    if strategy == "raw":
        return {"data": message}, ""

    elif strategy == "delimited":
        return parse_delimited_message(message, config)

    elif strategy == "json_dict":
        return parse_json_dict_message(message, config)

    elif strategy == "json_array":
        return parse_json_array_message(message, config)

    return {}, f"Unknown parsing strategy: {strategy}"

def is_valid_value(value: Any, type_hint: str) -> Tuple[bool, str]:
    """Validate value type against type hint and return error message if not valid

    Args:
        value: The value to validate
        type_hint: String from config like "float", "str", "int", "bool", etc.
    """
    # Validate type
    if type_hint.endswith("[]"):
        if not isinstance(value, (list, tuple)):
            return False, f"Field should be an array"
    elif type_hint == "float" and not isinstance(value, (int, float)):
        return False, f"Field should be a number"
    elif type_hint == "int" and not isinstance(value, int):
        return False, f"Field should be an integer"
    elif type_hint == "bool" and not isinstance(value, bool):
        return False, f"Field should be a boolean"

    return True, ""

def parse_delimited_message(message: bytes, config: dict) -> Tuple[dict, str]:
    """Parse delimited message into fields

    Args:
        message: Raw message bytes
        config: Config dict containing delimiter and subtopic definitions

    Returns:
        tuple[dict, str]: (parsed_fields, error_message)
    """
    try:

        # Decode and split the message
        text = message.decode('utf-8').strip()
        fields = text.split(config["delimiter"])
        result = {}

        # Process each subtopic
        for subtopic_name, subtopic_config in config["subtopics"].items():
            field_id = subtopic_config["field_id"]

            # Check field exists
            if isinstance(field_id, int) and field_id >= len(fields):
                return {}, f"Field index {field_id} out of range for message with {len(fields)} fields"

            # Get the raw value
            raw_value = fields[field_id]

            # Convert according to type
            try:
                if subtopic_config["type"] == "float":
                    value = float(raw_value)
                elif subtopic_config["type"] == "int":
                    value = int(raw_value)
                elif subtopic_config["type"] == "bool":
                    value = raw_value.lower() in ("true", "1", "t", "yes")
                elif subtopic_config["type"] == "str":
                    value = str(raw_value)
                else:
                    return {}, f"Unsupported delimited field type: {subtopic_config['type']}"

                result[subtopic_name] = value

            except ValueError as e:
                return {}, f"Error converting field {field_id} to {subtopic_config['type']}: {str(e)}"

        return result, ""

    except UnicodeDecodeError as e:
        return {}, f"Invalid UTF-8 in message: {str(e)}"

def parse_json_dict_message(message: bytes, config: dict) -> Tuple[dict, str]:
    """Parse JSON dictionary message

    Args:
        message: Raw message bytes
        config: Config dict containing subtopic definitions

    Returns:
        tuple[dict, str]: (parsed_fields, error_message)
    """
    try:

        # Parse JSON
        data = json.loads(message)
        if not isinstance(data, dict):
            return {}, "JSON message is not a dictionary"

        result = {}

        # Process each subtopic
        for subtopic_name, subtopic_config in config["subtopics"].items():
            field_id = subtopic_config["field_id"]  # This will be a string key for JSON dict

            # Check field exists
            if field_id not in data:
                return {}, f"Field '{field_id}' not found in JSON message"

            # Get the raw value
            raw_value = data[field_id]

            is_valid, error = is_valid_value(raw_value, subtopic_config["type"])
            if not is_valid:
                return {}, f"Field '{field_id}' is not a valid {subtopic_config['type']}: {error}"

            result[subtopic_name] = raw_value

        return result, ""

    except json.JSONDecodeError as e:
        return {}, f"Invalid JSON: {str(e)}"

def parse_json_array_message(message: bytes, config: dict) -> Tuple[dict, str]:
    """Parse JSON array message

    Args:
        message: Raw message bytes
        config: Config dict containing subtopic definitions

    Returns:
        tuple[dict, str]: (parsed_fields, error_message)
    """
    try:

        # Parse JSON
        data = json.loads(message)
        if not isinstance(data, (list, tuple)):
            return {}, "JSON message is not an array"

        result = {}

        # Process each subtopic
        for subtopic_name, subtopic_config in config["subtopics"].items():
            field_id = subtopic_config["field_id"]  # This will be an integer index

            # Check field exists
            if not isinstance(field_id, int):
                return {}, f"Field ID must be an integer for JSON array messages, got '{field_id}'"
            if field_id >= len(data):
                return {}, f"Field index {field_id} out of range for message with {len(data)} elements"

            # Get the raw value
            raw_value = data[field_id]

            # Validate type
            is_valid, error = is_valid_value(raw_value, subtopic_config["type"])
            if not is_valid:
                return {}, f"Field '{field_id}' is not a valid {subtopic_config['type']}: {error}"

            result[subtopic_name] = raw_value

        return result, ""

    except json.JSONDecodeError as e:
        return {}, f"Invalid JSON: {str(e)}"

def setup_publishers(config: dict) -> Tuple[Dict[str, dict], str]:
    """Set up publishers for all topics and subtopics

    Args:
        config: Complete node configuration

    Returns:
        tuple[dict, str]: (publishers_dict, error_message)
        publishers_dict structure:
        {
            "topic_name": {
                "raw": rospy.Publisher,  # For raw strategy
                "subtopics": {  # For other strategies
                    "subtopic_name": rospy.Publisher
                }
            }
        }
    """
    publishers = {}

    for topic_name, topic_config in config["topics"].items():
        publishers[topic_name] = {"subtopics": {}}

        if topic_config["parsing_strategy"] == "raw":

            # For raw strategy, create RawData publisher
            publishers[topic_name]["raw"] = rospy.Publisher(
                topic_name,
                RawData,
                queue_size=10
            )
        else:

            # For other strategies, create publishers for each subtopic
            for subtopic_name, subtopic_config in topic_config["subtopics"].items():
                msg_type = ROS_TYPE_MAP[subtopic_config["type"]]
                publishers[topic_name]["subtopics"][subtopic_name] = rospy.Publisher(
                    f"{topic_name}/{subtopic_name}",
                    msg_type,
                    queue_size=10
                )

    return publishers, ""


def publish_messages(topic_name: str, parsed_data: dict, publishers: dict,
                    parsing_strategy: str, topic_config: dict) -> str:
    """Publish parsed data to appropriate topics

    Args:
        topic_name: Name of the base topic
        parsed_data: Dict of subtopic -> value mappings
        publishers: Dict of publishers from setup_publishers
        topic_config: Dict of topic configuration values


    Returns:
        str: Error message (empty if successful)
    """

    if parsing_strategy == "raw":
        if "data" not in parsed_data:
            return "Raw message missing 'data' field"

        msg = RawData()
        msg.data = list(parsed_data["data"])  # Convert bytes to list of ints
        msg.data_direction = RawData.DATA_IN
        current_time = rospy.Time.now()
        msg.ds_header.io_time = current_time
        msg.header.stamp = current_time

        publishers[topic_name]["raw"].publish(msg)

    else:

        # Get the subtopic publishers for this topic
        topic_publishers = publishers[topic_name]["subtopics"]
        subtopics_config = topic_config["subtopics"]

        # Publish each parsed field to its subtopic
        for subtopic_name, value in parsed_data.items():
            if subtopic_name not in topic_publishers:
                return f"No publisher found for subtopic '{subtopic_name}'"

            # Convert value to ROS message
            msg, error = convert_to_ros_msg(
                value,
                subtopics_config[subtopic_name]["type"]
            )
            if error:
                return f"Error converting message for {subtopic_name}: {error}"

            topic_publishers[subtopic_name].publish(msg)

    return ""



@dataclass
class TopicStats:
    bytes_received: int = 0
    messages_processed: int = 0
    messages_published: int = 0
    parse_errors: int = 0
    publish_errors: int = 0
    last_message_time: float = 0
    buffer_size: int = 0
    message_rate: float = 0  # messages/sec
    error_rate: float = 0    # errors/sec

    # Moving window for rate calculation
    message_times: list = dc_field(default_factory=list)
    error_times: list = dc_field(default_factory=list)
    window_size: int = 10  # seconds

    def update_rates(self, current_time: float):
        """Update message and error rates using moving window"""
        # Remove old entries
        self.message_times = [t for t in self.message_times if t > current_time - self.window_size]
        self.error_times = [t for t in self.error_times if t > current_time - self.window_size]

        # Calculate rates
        self.message_rate = len(self.message_times) / self.window_size
        self.error_rate = len(self.error_times) / self.window_size

def log_topic_stats(topic_name: str, stats: TopicStats, print_stats: Bool = False):
    """Log statistics for a topic"""

    # Choose log function based on print_stats. logwarn will print to stdout
    log_func = rospy.logwarn if print_stats else rospy.loginfo

    log_func(f"Topic {topic_name} stats:")
    log_func(f"  Bytes received: {stats.bytes_received}")
    log_func(f"  Messages processed: {stats.messages_processed}")
    log_func(f"  Messages published: {stats.messages_published}")
    log_func(f"  Parse errors: {stats.parse_errors}")
    log_func(f"  Publish errors: {stats.publish_errors}")
    log_func(f"  Message rate: {stats.message_rate:.2f} msgs/sec")
    log_func(f"  Error rate: {stats.error_rate:.2f} errors/sec")
    log_func(f"  Current buffer size: {stats.buffer_size} bytes")

    if stats.last_message_time > 0:
        time_since_last = time.time() - stats.last_message_time
        if time_since_last > 5.0:  # Warning if no messages for 5 seconds
            rospy.logwarn(f"  No messages received for {time_since_last:.1f} seconds")

def main():
    rospy.init_node('network_data_capture', anonymous=True)

    # Load configuration
    config = rospy.get_param('~', None)
    if config is None:
        rospy.logwarn("No network_data_capture configuration found, stopping node")
        return

    is_valid, error = validate_config(config)
    if not is_valid:
        rospy.logerr(f"Invalid configuration: {error}")
        return

    # Create sockets
    sockets = {}
    buffers = {}
    stats = {}  # Track statistics per topic

    for topic_name, topic_config in config["topics"].items():
        sock, error = create_socket(topic_config["connection_type"], topic_config["port"])
        if error:
            rospy.logerr(f"Error creating socket for {topic_name}: {error}")
            return
        sockets[topic_name] = sock
        buffers[topic_name] = b""
        stats[topic_name] = TopicStats()

        rospy.loginfo(f"Created {topic_config['connection_type'].upper()} socket for {topic_name} "
                     f"on port {topic_config['port']}")

    # Set up publishers
    publishers, error = setup_publishers(config)
    if error:
        rospy.logerr(f"Error setting up publishers: {error}")
        return

    rospy.loginfo("Network Data Capture Node started")

    # Stats logging timer
    last_stats_time = time.time()
    stats_interval = rospy.get_param("~stats_interval", 60)  # Log stats every 60 seconds
    print_stats = rospy.get_param("~print_stats", False)

    while not rospy.is_shutdown():
        current_time = time.time()

        # Read available data
        received_data = read_available_data(sockets)

        # Process data for each topic
        for topic_name, (data, error) in received_data.items():
            topic_stats = stats[topic_name]

            if error:
                rospy.logerr(f"Error reading from {topic_name}: {error}")
                topic_stats.error_times.append(current_time)
                continue

            if data:  # Data received
                topic_stats.bytes_received += len(data)
                topic_stats.last_message_time = current_time

                # Get topic config
                topic_config = config["topics"][topic_name]

                # Append to buffer
                buffers[topic_name], error = append_to_buffer(buffers[topic_name], data)
                if error:
                    rospy.logerr(f"Buffer error for {topic_name}: {error}")
                    topic_stats.error_times.append(current_time)
                    continue

                topic_stats.buffer_size = len(buffers[topic_name])

                # Extract messages
                messages, buffers[topic_name], error = extract_messages(
                    buffers[topic_name],
                    topic_config["parsing_strategy"],
                    topic_config
                )
                if error:
                    rospy.logerr(f"Error extracting messages for {topic_name}: {error}")
                    topic_stats.error_times.append(current_time)
                    continue

                topic_stats.messages_processed += len(messages)

                # Process and publish each message
                for message in messages:
                    topic_stats.message_times.append(current_time)

                    parsed_data, error = parse_message(
                        message,
                        topic_config["parsing_strategy"],
                        topic_config
                    )
                    if error:
                        rospy.logerr(f"Error parsing message for {topic_name}: {error}")
                        topic_stats.parse_errors += 1
                        topic_stats.error_times.append(current_time)
                        continue

                    error = publish_messages(
                        topic_name,
                        parsed_data,
                        publishers,
                        topic_config["parsing_strategy"],
                        topic_config
                    )
                    if error:
                        rospy.logerr(f"Error publishing message for {topic_name}: {error}")
                        topic_stats.publish_errors += 1
                        topic_stats.error_times.append(current_time)
                        continue

                    topic_stats.messages_published += 1

            # Update rates for this topic
            topic_stats.update_rates(current_time)

        # Log stats periodically
        if current_time - last_stats_time >= stats_interval:
            rospy.loginfo("=== Network Data Capture Statistics ===")
            for topic_name, topic_stats in stats.items():
                log_topic_stats(topic_name, topic_stats, print_stats)
            last_stats_time = current_time

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
