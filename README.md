# Bikebus Battery
# Hardware:

- ESP32-C6
- Daly BMS (Daly SMART H SERIES L-Ion BMS 10S 36V, see below for details)
- TLIN2029A LIN tranceiver
- Optional: Adapter to go from SOIC-8 to DIP-8 to make handling of the small TLIN2029A chip easier
- Battery pack (I modified the orignal to keep the cells, the LEDs with button as well as the case with plug)
- Ebike like "Bulls SPORTSLITE Green Mover" to use the battery pack in

We use an ESP32-C6 to communicate with both the new BMS (see below) and the Bikebus. For the Bikebus this is specifically the display, which acts as the master and periodically asks for various parameters from battery and motor. We could also pretent to be the motor or the display, the protocoll is decoded, but that is not used here since this is the battery only. The electrical wiring essentially looks like the following, go to the simulator:
https://www.falstad.com/circuit/ 

 -> File -> Import from Text. Copy and paste the text from "Bikebus_Tranceiver_Leo_Vx.y.txt" or download the txt file and directly "open" it in the simulator.

Key for everything is the TLIN2029A tranceiver for the LIN bus. It can handle the high voltage directly and can be used to wake up the MCU when signals start flowing.
To make sure we turn the output of the battery off when there is no connection, we use a 3.6 V zener diode. In my case, when I connect the plug (the type is rosenberg), pin 3 is connected to GND (Pack minus). So I pull it high to Pack plus (30...42 V) trough 100 kOhm and measure if it is pulled low -> connected. Otherwise we command the BMS to turn the output off to avoid short circuits etc. Please check if the wiring is the same for you.

The downside is that, starting from a disconnected battery (=output off) we do not realibly notice if a plug gets connected as the BMS has the output turned off. So we then need to press the button on the battery once to trigger a "plug check" -> turns the output on -> checks for the signal -> stays on or turns back off.
The BMS is a low-side switch and the connection between pin 3 and 4 of the rosenberg plug happens outside the battery, so I can not simply supply my own logic high on plug pin 3 or 4 and measure it on the other pin. I am not sure how to check the plug without risking damage, so we need to press the button after inserting the battery (or attaching any other plug). The low-side switching has already killed 2 or 3 MCUs since it causes everything on the bike to be floating at ~20 V or so while the output is off. So whatever you do, always assume the pack minus to be FAR above save levels for the MCU.

You can also read all of my posts in this thread. Be aware that some details changed over time as I understood the system better, so make sure to read everything (or ask here/there) before damaging something.
https://www.pedelecforum.de/forum/index.php?threads/go-swissdrive-bikebus-service-adapter.94004/

# BMS
Of course we also need to connect the BMS itself to the battery cells. Just changing the BMS means the battery itself will work, but the display will not show the capacity nor the current. The contact would also always be live and since there are magnets in the plug, steel will get attracted and could short it. In the "Pinout Akku Balancing.jpg" you can find the pinout of the existing balancing plug. Please check to make sure this is correct for you too. Snip one wire at a time, insert a piece of heat shrink tube, solder it to the new BMS plug, slide the heat shrink over and apply heat to shrink/seal the connection. One wire at a time reduces the risk of short circuits.

*Daly SMART H SERIES L-Ion BMS 10S 36V for 55 dollar. I used the "BT+UART" one and then had to cut the BT lines to get UART (I did not have the right connector!). You should choose the BT+CAN version instead, it has both cables. The BT is nice to have, as you can check the battery status and change settings with your phone. We could also add a website to the MCU and let it do the same things, but that is a bit of effort to implement.
https://bmsdaly.com/products/daly-smart-lifepo4-bms-4s-8s-16s-24s-12v-24v-48v-li-ion-10s-36v-13s-48v-20s-72v-40a-60a-bms-for-lithium-18650-battery-1


# Software
With this code, you can replace a battery from an ebike with the bikebus communication system (like the Bulls SPORTSLITE Green Mover). 
The BMS in the new battery is from Daly*, so that is what the code expects to communicate with. But you could use any other BMS with or without communication with fairly minimal adaptation. All you really need is roughly the state of charge (SOC) of the battery and, if you want to look at it, the current draw. The other values are not really usefull.

All you need to do, on the software side of things, is upload the 7 files to an ESP32-C6 via the arduino IDE:
  1. Bikebus_from_Daly_2025_10_03.ino
  2. BikeBusProtocol.cpp
  3. BikeBusProtocol.h
  4. DalyProtocol.cpp
  5. DalyProtocol.h
  6. Kalman.h
  7. Serial_Commands.ino

