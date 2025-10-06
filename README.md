# Bikebus_Battery
With this code, you can replace a battery from an ebike with the bikebus communication system (like the Bulls SPORTSLITE Green Mover). 
The BMS in the new battery is from Daly, so that is what the code expects to communicate with. But you could use any other BMS with or without communication with fairly minimal adaptation. All you really need is roughly the state of charge (SOC) of the battery and, if you want to look at it, the current draw. The other values are not really usefull.

All you need to do, on the software side of things, is upload the 7 files to an ESP32-C6 via the arduino IDE:
  1. Bikebus_from_Daly_2025_10_03.ino
  2. BikeBusProtocol.cpp
  3. BikeBusProtocol.h
  4. DalyProtocol.cpp
  5. DalyProtocol.h
  6. Kalman.h
  7. Serial_Commands.ino

Hardware looks like the following. Go to this simulator -> File -> Import from Text:
https://www.falstad.com/circuit/
Copy and paste the text from "Bikebus_Tranceiver_Leo_V1.3.txt" or download the txt file directly "open" it in the simulator.
Key for everything is the TLIN2029A tranceiver for the LIN bus. It can handle the high voltage directly and can be used to wake up the MCU when signals start flowing.
To make sure we turn the output of the battery off when there is no connection, we use a 3.6 V zener diode. In my case, when I connect the plug (the type is rosenberg), pin 3 is connected to GND (Pack minus). So I pull it high to Pack plus (30...42 V) trough 100 kOhm and measure if it is pulled low -> connected. Otherwise we command the BMS to turn the output off to avoid short circuits etc.
The downside is that, starting from a disconnected battery (=output off) we do not realibly notice if a plug gets connected as the BMS has the output turned off. So we then need to press the button on the battery once to trigger a "plug check" -> turns the output on -> checks for the signal -> stays on or turns back off.
The BMS is a low-side switch and the connection between pin 3 and 4 happens outside the battery, so I can not simply supply my own logic high on plug pin 3 or 4 and measure it on the other pin. I am not sure how to check the plug without risking damage and you need to press the button after inserting the battery (or attaching a plug).



