import brahe

from .constants import seconds_in_week


def gps_week_milliseconds_to_brahe_epoch(gps_week, gps_milliseconds):
    gps_epoch = brahe.epoch.Epoch("1980-01-06")  # gps time starting epoch
    gps_epoch += gps_week * seconds_in_week
    gps_epoch += gps_milliseconds / 1000.0

    return gps_epoch
