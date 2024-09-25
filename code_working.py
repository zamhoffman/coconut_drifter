import temp_sense
import gps_info
from time import sleep

print("hello world")

while True:
    temp = temp_sense.get_temp_measurement(1000)
    timestamp, latitude, longitude, altitude = gps_info.get_curr_info()
    print("temp is: ", temp)

    print(
            "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
                timestamp[0],  # Grab parts of the time from the
                timestamp[1],  # struct_time object that holds
                timestamp[2],  # the fix time.  Note you might
                timestamp[3],  # not get all data like year, day,
                timestamp[4],  # month!
                timestamp[5],
            )
        )
    print(
            "Precise Latitude: {} degs, {:2.4f} mins".format(
                latitude[0], latitude[1]
            )
        )
    print(
            "Precise Longitude: {} degs, {:2.4f} mins".format(
                longitude[0], longitude[1]
            )
        )
    if altitude is not None:
            print("Altitude: {} meters".format(altitude))
    sleep(5)
