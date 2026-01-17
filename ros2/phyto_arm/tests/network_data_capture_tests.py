import socket
import time
import unittest
from unittest.mock import Mock, patch

from phyto_arm import network_data_capture
from phyto_arm.network_data_capture import (
    append_to_buffer, convert_to_ros_msg, extract_messages,
    parse_delimited_message, parse_json_array_message, parse_json_dict_message,
    validate_config, validate_subtopic_config, validate_topic_config
)
from wr2_msgs.msg import RawData


class TestNetworkDataCaptureValidation(unittest.TestCase):
    def test_validate_config_valid(self):
        config = {
            "topics": {
                "test_topic": {
                    "connection_type": "udp",
                    "port": 8080,
                    "parsing_strategy": "json_dict",
                    "subtopics": {
                        "value": {
                            "field_id": "data",
                            "type": "float"
                        }
                    }
                }
            }
        }
        # No exception should be raised
        validate_config(config)

    def test_validate_config_missing_topics(self):
        config = {}
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_config(config)
        self.assertIn("must contain 'topics'", str(context.exception))

    def test_validate_topic_config_valid(self):
        topic_config = {
            "connection_type": "udp",
            "port": 8080,
            "parsing_strategy": "json_dict",
            "subtopics": {
                "value": {
                    "field_id": "data",
                    "type": "float"
                }
            }
        }
        # No exception should be raised
        validate_topic_config("test_topic", topic_config)

    def test_validate_topic_config_missing_field(self):
        topic_config = {
            "connection_type": "udp",
            "parsing_strategy": "json_dict",
            "subtopics": {}
        }
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_topic_config("test_topic", topic_config)
        self.assertIn("Missing required field 'port'", str(context.exception))

    def test_validate_topic_config_invalid_connection_type(self):
        topic_config = {
            "connection_type": "invalid",
            "port": 8080,
            "parsing_strategy": "json_dict",
            "subtopics": {}
        }
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_topic_config("test_topic", topic_config)
        self.assertIn("Invalid connection_type", str(context.exception))

    def test_validate_topic_config_missing_delimiter(self):
        topic_config = {
            "connection_type": "udp",
            "port": 8080,
            "parsing_strategy": "delimited",
            "subtopics": {}
        }
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_topic_config("test_topic", topic_config)
        self.assertIn("Delimiter required", str(context.exception))

    def test_validate_topic_config_delimited_with_array_type(self):
        topic_config = {
            "connection_type": "udp",
            "port": 8080,
            "parsing_strategy": "delimited",
            "delimiter": ",",
            "subtopics": {
                "values": {
                    "field_id": 0,
                    "type": "float[]"
                }
            }
        }
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_topic_config("test_topic", topic_config)
        self.assertIn("Arrays are not a supported field type for delimited string parsing", str(context.exception))

    def test_validate_subtopic_config_valid(self):
        topic_config = {
            "parsing_strategy": "json_dict",
            "subtopics": {
                "test_subtopic": {
                    "field_id": "data",
                    "type": "float"
                }
            }
        }
        subtopic_config = topic_config["subtopics"]["test_subtopic"]
        # No exception should be raised
        validate_subtopic_config(subtopic_config, topic_config["parsing_strategy"])

    def test_validate_subtopic_config_missing_field(self):
        topic_config = {
            "parsing_strategy": "json_dict",
            "subtopics": {
                "test_subtopic": {
                    "field_id": "data"
                }
            }
        }
        subtopic_config = topic_config["subtopics"]["test_subtopic"]
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_subtopic_config(subtopic_config, topic_config["parsing_strategy"])
        self.assertIn("Missing required field 'type'", str(context.exception))

    def test_validate_subtopic_config_invalid_type(self):
        topic_config = {
            "parsing_strategy": "json_dict",
            "subtopics": {
                "test_subtopic": {
                    "field_id": "data",
                    "type": "invalid_type"
                }
            }
        }
        subtopic_config = topic_config["subtopics"]["test_subtopic"]
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_subtopic_config(subtopic_config, topic_config["parsing_strategy"])
        self.assertIn("Invalid type", str(context.exception))


class TestNetworkDataCaptureMessageProcessing(unittest.TestCase):
    def test_publish_raw_packet(self):
        stamp = network_data_capture.rospy.Time(12, 345)
        payload = b'\x00\x80\xff'
        publisher = Mock()
        with patch.object(network_data_capture.rospy.Time, 'now', return_value=stamp):
            network_data_capture.publish_messages(
                'input', {'data': payload}, {'input': {'raw': publisher}}, 'raw', {})

        packet, = publisher.publish.call_args[0]
        self.assertIsInstance(packet, RawData)
        self.assertEqual(packet.io_stamp, stamp)
        self.assertEqual(packet.data_direction, RawData.DIRECTION_DATA_IN)
        self.assertEqual(bytes(packet.data), payload)

    def test_convert_to_ros_msg_float(self):
        msg = convert_to_ros_msg(42.5, "float")
        self.assertEqual(msg.data, 42.5)

    def test_convert_to_ros_msg_string(self):
        msg = convert_to_ros_msg("test", "str")
        self.assertEqual(msg.data, "test")

    def test_convert_to_ros_msg_int(self):
        msg = convert_to_ros_msg(42, "int")
        self.assertEqual(msg.data, 42)

    def test_convert_to_ros_msg_bool(self):
        msg = convert_to_ros_msg(True, "bool")
        self.assertEqual(msg.data, True)

    def test_convert_to_ros_msg_float_array(self):
        msg = convert_to_ros_msg([1.0, 2.0, 3.0], "float[]")
        self.assertEqual(list(msg.data), [1.0, 2.0, 3.0])

    def test_convert_to_ros_msg_int_array(self):
        msg = convert_to_ros_msg([1, 2, 3], "int[]")
        self.assertEqual(list(msg.data), [1, 2, 3])

    def test_convert_to_ros_msg_bool_array(self):
        msg = convert_to_ros_msg([True, False, True], "bool[]")
        self.assertEqual(list(msg.data), [1, 0, 1])

    def test_convert_to_ros_msg_type_mismatch(self):
        with self.assertRaises(network_data_capture.ConversionError):
            convert_to_ros_msg("not_a_number", "float")

    def test_append_to_buffer(self):
        buffer = append_to_buffer(b"hello", b" world")
        self.assertEqual(buffer, b"hello world")

    def test_append_to_buffer_overflow(self):
        with self.assertRaises(BufferError):
            append_to_buffer(b"hello", b" world", max_size=10)

    def test_extract_messages_raw(self):
        buffer = b"line1\nline2\nline3"
        messages, remaining = extract_messages(buffer, "raw", {})
        self.assertEqual(messages, [b"line1", b"line2"])

        # Remaining because raw only splits on newlines
        self.assertEqual(remaining, b"line3")

    def test_extract_messages_delimited(self):
        buffer = b"a,b,c\nd,e,f\ng,h"
        config = {"delimiter": ",", "use_regex_delimiter": False}
        messages, remaining = extract_messages(buffer, "delimited", config)
        self.assertEqual(messages, [b"a,b,c", b"d,e,f"])
        self.assertEqual(remaining, b"g,h")

    def test_extract_messages_delimited_with_regex(self):
        buffer = b"a, b,  c  d\nd, e,  f  e\ng, h"
        config = {"delimiter": ",?\\s+", "use_regex_delimiter": True}
        messages, remaining = extract_messages(buffer, "delimited", config)
        self.assertEqual(messages, [b"a, b,  c  d", b"d, e,  f  e"])
        self.assertEqual(remaining, b"g, h")

    def test_extract_messages_json_dict(self):
        buffer = b'{"a":1}\n{"b":2}\n{"c":'
        messages, remaining = extract_messages(buffer, "json_dict", {})
        self.assertEqual(len(messages), 2)
        self.assertEqual(remaining, b'{"c":')

    def test_extract_messages_invalid_strategy(self):
        buffer = b"some data"
        with self.assertRaises(network_data_capture.ParsingError):
            extract_messages(buffer, "invalid_strategy", {})

    def test_parse_delimited_message(self):
        message = b"42.5,test,1"
        config = {
            "delimiter": ",",
            "use_regex_delimiter": False,
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }
        result = parse_delimited_message(message, config)
        self.assertEqual(result["value"], 42.5)
        self.assertEqual(result["name"], "test")
        self.assertEqual(result["flag"], True)

    def test_parse_delimited_message_with_regex(self):
        message = b"42.5, test  1"
        config = {
            "delimiter": ",?\\s+",
            "use_regex_delimiter": True,
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }
        result = parse_delimited_message(message, config)
        self.assertEqual(result["value"], 42.5)
        self.assertEqual(result["name"], "test")
        self.assertEqual(result["flag"], True)

    def test_parse_delimited_message_field_out_of_range(self):
        message = b"42.5,test"
        config = {
            "delimiter": ",",
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }

        with self.assertRaises(network_data_capture.ParsingError):
            parse_delimited_message(message, config)

    def test_parse_json_dict_message(self):
        message = b'{"value": 42.5, "name": "test", "flag": true}'
        config = {
            "subtopics": {
                "value": {"field_id": "value", "type": "float"},
                "name": {"field_id": "name", "type": "str"},
                "flag": {"field_id": "flag", "type": "bool"}
            }
        }
        result = parse_json_dict_message(message, config)
        self.assertEqual(result["value"], 42.5)
        self.assertEqual(result["name"], "test")
        self.assertEqual(result["flag"], True)

    def test_parse_json_dict_message_missing_field(self):
        message = b'{"value": 42.5, "flag": true}'
        config = {
            "subtopics": {
                "value": {"field_id": "value", "type": "float"},
                "name": {"field_id": "name", "type": "str"},
                "flag": {"field_id": "flag", "type": "bool"}
            }
        }
        with self.assertRaises(network_data_capture.ParsingError):
            parse_json_dict_message(message, config)

    def test_parse_json_array_message(self):
        message = b'[42.5, "test", true]'
        config = {
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }
        result = parse_json_array_message(message, config)
        self.assertEqual(result["value"], 42.5)
        self.assertEqual(result["name"], "test")
        self.assertEqual(result["flag"], True)

    def test_parse_json_array_message_index_out_of_range(self):
        message = b'[42.5, "test"]'
        config = {
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }
        with self.assertRaises(network_data_capture.ParsingError):
            parse_json_array_message(message, config)

    @patch('socket.socket')
    def test_create_socket_udp(self, mock_socket):
        mock_socket_instance = Mock()
        mock_socket.return_value = mock_socket_instance

        network_data_capture.create_socket("udp", 8080)

        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_DGRAM)
        mock_socket_instance.bind.assert_called_with(("0.0.0.0", 8080))

    @patch('socket.socket')
    def test_create_socket_tcp(self, mock_socket):
        mock_socket_instance = Mock()
        mock_socket.return_value = mock_socket_instance

        network_data_capture.create_socket("tcp", 8080)

        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_STREAM)
        mock_socket_instance.bind.assert_called_with(("0.0.0.0", 8080))
        mock_socket_instance.listen.assert_called_with(1)

    @patch('socket.socket')
    def test_create_socket_error(self, mock_socket):
        mock_socket.side_effect = socket.error("Test socket error")

        with self.assertRaises(network_data_capture.SocketError):
            network_data_capture.create_socket("udp", 8080)


class TestTopicStats(unittest.TestCase):
    def test_update_rates(self):
        stats = network_data_capture.TopicStats()

        # Add some message times
        current_time = time.time()
        stats.message_times = [
            current_time - 9,
            current_time - 7,
            current_time - 5,
            current_time - 3,
            current_time - 1
        ]

        # Add some error times
        stats.error_times = [
            current_time - 8,
            current_time - 4
        ]

        # Update rates
        stats.update_rates(current_time)

        # Check rates
        self.assertEqual(stats.message_rate, 0.5)  # 5 messages / 10 seconds
        self.assertEqual(stats.error_rate, 0.2)    # 2 errors / 10 seconds

        # Add an old message that should be removed
        stats.message_times.append(current_time - 15)
        stats.update_rates(current_time)

        # The old message should be removed
        self.assertEqual(stats.message_rate, 0.5)  # Still 5 messages / 10 seconds
