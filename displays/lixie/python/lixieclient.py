#!/usr/bin/env python3
# Client to display values on lixie
# Author: Michel Verlaan (michel.verl@gmail.com)
#

import time
import datetime
from rpi_ws281x import Color
from plixie import Element
import Adafruit_DHT

DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4

if __name__ == '__main__':

    element1 = Element(0)
    element2 = Element(1)
    element3 = Element(2)
    element4 = Element(3)

    try:

        while True:
            # Get current time
            currentTime = datetime.datetime.now()

            # Convert to string to get individual numbers
            currentHour = currentTime.strftime("%H")
            currentMinute = currentTime.strftime("%M")

            # Get humidity and temperature
            humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)

            # Convert to string to get individual numbers
            str_humidity = str(humidity)
            str_temperature = str(temperature)

            # Display time
            element1.Clear()
            element2.Clear()
            element3.Clear()
            element4.Clear() 
            
            element1.SetElement(int(currentHour[0]), Color(0, 255, 0))
            element2.SetElement(int(currentHour[1]), Color(0, 255, 0))
            element3.SetElement(int(currentMinute[0]), Color(255, 255, 255))
            element4.SetElement(int(currentMinute[1]), Color(255, 255, 255))
            time.sleep(10)
            
            # Display temperature
            element1.Clear()
            element2.Clear()
            element3.Clear()
            element4.Clear() 

            element1.SetElement(int(str_temperature[0]), Color(255, 255, 255))
            element2.SetElement(int(str_temperature[1]), Color(255, 255, 255))
            element3.SetElement(0, Color(0, 0, 0))
            element4.SetElement(int(str_temperature[3]), Color(255, 255, 255))
            time.sleep(10)

            # Display humidity
            element1.Clear()
            element2.Clear()
            element3.Clear()
            element4.Clear() 

            element1.SetElement(int(str_humidity[0]), Color(0, 255, 0))
            element2.SetElement(int(str_humidity[1]), Color(0, 255, 0))
            element3.SetElement(0, Color(0, 0, 0))
            element4.SetElement(int(str_humidity[3]), Color(0, 255, 0))
            time.sleep(10)

    except KeyboardInterrupt:
        element1.Clear()
        element2.Clear()
        element3.Clear()
        element4.Clear()
