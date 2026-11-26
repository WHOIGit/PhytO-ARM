import unittest

from unittest.mock import Mock, patch

from phyto_arm import seatrac_starlink_manager as manager
from rclpy.parameter import Parameter
from seatrac.msg import OutletStatus


class TestStarlinkReminder(unittest.TestCase):
    def test_active_outlet_sends_reminder_when_timer_fires(self):
        alerts = [{'type': 'slack', 'url': 'https://example.invalid/alerts'}]
        node = Mock()
        node.get_parameters_by_prefix.side_effect = {
            'alerts': {'0.' + k: Parameter('0.' + k, value=v) for k, v in alerts[0].items()},
            'soc_threshold': {},
        }.__getitem__
        node.get_parameter_or.side_effect = {
            'shutdown_reminder': Parameter('shutdown_reminder', value=1),
            'scheduled_on': Parameter('scheduled_on'),
        }.__getitem__
        with patch.object(manager, 'send_alerts') as alert:
            manager.configure_node(node)
            subscribers = {call.args[1]: call.args[2] for call in node.create_subscription.call_args_list}
            subscribers['/seatrac/outlet/starlink/status'](OutletStatus(is_active=True))

            seconds, callback = node.create_timer.call_args.args
            self.assertEqual(seconds, 1)
            callback()
            node.destroy_timer.assert_called_once_with(node.create_timer.return_value)
            alert.assert_called_once_with(alerts, 'Starlink has been active for a while')
