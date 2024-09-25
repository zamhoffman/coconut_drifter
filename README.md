# coconut_drifter

This project utilizes Adafruit's [Feather M4](https://www.adafruit.com/product/3857) development board. We utilize adafruit's [CircuitPython](https://circuitpython.org/) platform for implementation, using 8.x.x [version](https://circuitpython.org/board/feather_m4_express/) and its corresponding 8.x [libraries](https://circuitpython.org/libraries).

We use three modules: 

## [1. AD7124-4 ADC](https://www.tindie.com/products/nhbsystems/24-bit-analog-sensor-featherwing/)
This is a featherwing board that contains a 4 channel 24 bit ADC. We utilize two channels to measure voltages accross a [PT 1000 RTD](https://evosensors.com/) and a precision resistor. We modify a micropython [library](https://github.com/NHBSystems/micropython_AD7124) written by [@jjuliano77](https://github.com/jjuliano77), see [circuitpython_AD7124.py](https://github.com/zamhoffman/coconut_drifter/blob/main/circuitpython_AD7124.py).
We implement this module in the helper file [temp_sense.py](https://github.com/zamhoffman/coconut_drifter/blob/main/temp_sense.py).

## [2. GPS Featherwing](https://www.adafruit.com/product/3133)
This is a featherwing board that contains a GPS module that uilizes the [adafruit_gps.mpy](https://circuitpython.org/libraries) library. 
We implement this module in the helper file [gps_info.py](https://github.com/zamhoffman/coconut_drifter/blob/main/gps_info.py).

## [3. RockBLOCK 9603](https://www.adafruit.com/product/4521)
This board contains a module that connects with the IRIDIUM constellation and utilizes the dafruit_rockblock library.
We implement this module in the helper file [transmit_data.py](https://github.com/zamhoffman/coconut_drifter/blob/main/transmit_data.py).

****Importantly, we do not utilize the default RX and TX UART pins, as the GPS Featherwing already uses those. The helper file utilizes digital pin 5 and 6 as TX and RX, respecively.**

# Basic API

## Import helper files
```python
import temp_sense
import gps_info
import transmit_data.py
```
## Temperature reading
To get a temperature reading, you can utilize the temp_sense.get_temp_measurement(..) method:
```python
temp = temp_sense.get_temp_measurement(REF_val)
```
Where
| Arg           |  Description    |
| -----------   | --------------- |
|`REF_val`      | reference resistor value |

So, to read the temperature off of a pt1000 RTD, with a precision resistance of 100 ohms...
```python
temp = temp_sense.get_temp_measurement(100)
print("temp is: ", temp," degrees C")
```
## GPS reading
To get GPS measurements, you can utilize the gps_info.get_curr_info() method:
```python
timestamp, latitude, longitude, altitude = gps_info.get_curr_info()
```
Where
| Return value           |  Description    |
| -----------   | --------------- |
|`timestamp`     | a list *[month, day, year, hour, minute, seconds]* |
|`latitude`     | a list *[latitude_degrees, latitude_minutes]* |
|`longitude`     | a list *[longitude_degrees, longitude_minutes]* |
|`altitude`     | altitude in meters |

## Data transmission
To get send data via Iridium, you can uitilize the transmit_data.send_data(..) method:
```python
transmit_data.send_data(out_data)
```
Where
| Arg           |  Description    |
| -----------   | --------------- |
|`out_data`      | a list of doubles to be transmitted |
