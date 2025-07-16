#!/usr/bin/env python3
import functools
import importlib
import io
import os
import threading
import time

import rospy
from std_msgs.msg import String

from aiohttp import web


response_values = {}
metadata_subscribers = {}  # Track active metadata subscribers
discovery_lock = threading.Lock()  # Thread safety for topic discovery

def capture_field_value(name, field, msg):
    # Since the data topic is dynamic, we need to re-interpret the data as the
    # correct type. This is a bit hacky, but described here:
    # http://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
    pkg, _, clsname = msg._connection_header['type'].partition('/')
    msg_class = getattr(importlib.import_module(f'{pkg}.msg'), clsname)
    if not hasattr(msg_class, field):
        rospy.logerr(f'Field {field} not found in msg type {msg_class}. Check config {name}.')
    buf = io.BytesIO()
    msg.serialize(buf)
    parsed = msg_class().deserialize(buf.getvalue())
    field_value = getattr(parsed, field)
    response_values[name] = field_value


def capture_metadata_value(metadata_key, msg):
    """Callback for arm_sipper metadata topics that publish String messages directly."""
    if msg.data:  # Only include non-empty values
        response_values[metadata_key] = msg.data
        rospy.logdebug(f"Updated metadata {metadata_key}: {msg.data}")
    else:  # Remove field if empty (cleared)
        if metadata_key in response_values:
            del response_values[metadata_key]
            rospy.logdebug(f"Removed metadata {metadata_key} (cleared)")


def discover_and_subscribe_metadata_topics():
    """Discover new arm_sipper metadata topics and subscribe to them."""
    try:
        with discovery_lock:
            # Get list of all published topics
            published_topics = rospy.get_published_topics()
            
            # Find arm_sipper metadata topics
            metadata_topic_prefix = '/arm_sipper/sample_metadata/'
            for topic_name, topic_type in published_topics:
                if topic_name.startswith(metadata_topic_prefix) and topic_name not in metadata_subscribers:
                    # Extract metadata key from topic name
                    metadata_key = topic_name[len(metadata_topic_prefix):]
                    
                    # Only subscribe to String topics (which is what arm_sipper publishes)
                    if topic_type == 'std_msgs/String':
                        rospy.loginfo(f"Discovered new metadata topic: {topic_name} -> {metadata_key}")
                        
                        # Create subscriber for this metadata topic
                        subscriber = rospy.Subscriber(
                            topic_name, 
                            String, 
                            functools.partial(capture_metadata_value, metadata_key)
                        )
                        metadata_subscribers[topic_name] = subscriber
                        
                        # Don't initialize empty values - only show fields with actual data
                            
    except Exception as e:
        rospy.logwarn(f"Error during metadata topic discovery: {e}")


def periodic_topic_discovery():
    """Periodically discover new metadata topics."""
    discovery_interval = rospy.get_param('~metadata_discovery_interval', 5.0)  # seconds
    
    while not rospy.is_shutdown():
        discover_and_subscribe_metadata_topics()
        time.sleep(discovery_interval)


app = web.Application()
routes = web.RouteTableDef()


@routes.get('/')
async def index(request):
    return web.json_response(response_values)


app.add_routes(routes)


def main():
    # Note that rospy spawns threads for message dispatch, so we don't have to
    rospy.init_node('web_node')

    # Handle legacy field_map configuration (optional for backward compatibility)
    field_map = rospy.get_param('~field_map', {})
    for name, config in field_map.items():
        if 'environment' in config:
            if 'topic' in config:
                rospy.logerr(f'Config {name} invalid. Mixing topic and environment not supported')
            var_name = config['environment']
            if var_name in os.environ:
                response_values[name] = os.environ[var_name]
            elif 'default' in config:
                rospy.logwarn(f'Env var {var_name} not found, switching to default for config {name}')
                response_values[name] = config['default']
            else:
                rospy.logerr(f'Config {name} invalid. Env var {var_name} not found and no default provided')
        elif 'default' in config:
            response_values[name] = config['default']
            # Subscriber optional; skip if no topic. Makes default permanent
            if 'topic' in config:
                if 'topic_field' not in config:
                    rospy.logerr(f'Config {name} invalid, topic_field required when topic used')
                rospy.Subscriber(config['topic'], rospy.AnyMsg,
                                functools.partial(capture_field_value, name, config['topic_field']))
        else:
            rospy.logerr(f'Config {name} invalid, must have "default" or "environment" set')

    # Start automatic discovery of arm_sipper metadata topics
    rospy.loginfo("Starting automatic discovery of arm_sipper metadata topics")
    discovery_thread = threading.Thread(target=periodic_topic_discovery, daemon=True)
    discovery_thread.start()
    
    # Do initial discovery
    discover_and_subscribe_metadata_topics()
    
    web.run_app(app, port=8098)


if __name__ == '__main__':
    main()
