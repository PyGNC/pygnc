import unittest

from .. import context
from pygnc.common import transformations  # type: ignore


class TestTransformations(unittest.TestCase):
    def test_gps_base_time_to_brahe_time(self):
        gps_epoch = transformations.gps_week_milliseconds_to_brahe_epoch(0, 0)
        self.assertEqual(gps_epoch.year(), 1980)
        self.assertEqual(gps_epoch.day(), 6)
        self.assertEqual(gps_epoch.hour(), 0)

    def test_gps_to_brahe_time_13nov2023(self):
        print(
            "sofa generates a 'dubious year' warning for years after 2023 because of leap second uncertainty."
        )
        print("There has not been a leap second since 2016, so this is not a concern")
        gps_epoch = transformations.gps_week_milliseconds_to_brahe_epoch(
            2288, 112947 * 1000
        )

        self.assertEqual(gps_epoch.year(), 2023)
        self.assertEqual(gps_epoch.month(), 11)
        self.assertEqual(gps_epoch.day(), 13)
        self.assertEqual(gps_epoch.hour(), 7)
        self.assertEqual(gps_epoch.minute(), 22)
        self.assertEqual(gps_epoch.second(), 9)
