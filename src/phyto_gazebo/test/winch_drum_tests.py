import os
import sys
import unittest

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from winch_drum import line_length


class TestWinchDrum(unittest.TestCase):
    def test_encoder_to_line_length(self):
        # One spool turn takes 60 motor turns in the example configuration
        for turns in (-1, 0, 1):
            with self.subTest(turns=turns):
                length = line_length(turns * 60 * 8192, 1.32, 60, 0.6604)
                self.assertAlmostEqual(length, 1.32 + turns * 0.6604)
        self.assertEqual(line_length(-3 * 60 * 8192, 1.32, 60, 0.6604), 0)
