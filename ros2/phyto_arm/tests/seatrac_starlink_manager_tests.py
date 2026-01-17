import unittest
from unittest.mock import patch

from phyto_arm import seatrac_starlink_manager as manager
from rospy.timer import TimerEvent
from seatrac.msg import OutletStatus


class TestStarlinkReminder(unittest.TestCase):
    def test_active_outlet_sends_reminder_when_timer_fires(self):
        alerts = [{'type': 'slack', 'url': 'https://example.invalid/alerts'}]
        params = {'~shutdown_reminder': 1, '/alerts': alerts}
        with patch.object(manager, 'rospy') as ros, \
                patch.object(manager, 'send_alerts') as alert:
            ros.get_param.side_effect = params.get
            manager.main()
            subscribers = {call.args[0]: call.args[2] for call in ros.Subscriber.call_args_list}
            subscribers['/seatrac/outlet/starlink/status'](OutletStatus(is_active=True))

            callback = ros.Timer.call_args.args[1]
            ros.Timer.assert_called_once_with(ros.Duration.return_value, callback, oneshot=True)
            callback(TimerEvent(None, None, None, None, None))
            alert.assert_called_once_with(alerts, 'Starlink has been active for a while')
