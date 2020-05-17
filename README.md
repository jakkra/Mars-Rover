# Rover
Controller for driving the Rover found here [https://github.com/jakkra/RoverController](https://github.com/jakkra/RoverController).

## CAD model
The whole Fusion 360 project can be found in the CAD folder, download the .f3z file and open it in desktop Fusion 360. 

### Wheels
There are two options for wheeles, either original Curiosity style wheels that print in one piece. Or Mars 2020 style wheels that print in two parts where outer part can be printed in flexibale plastic such as TPU. The rover_2020_wheel_flex.f3d wheels are preferred if your printer can print in TPU as they provide way more grip.

## Driving
There are 3 ways to drive the Rover right now:
- Websockets (website hosted on the Rover, see data folder)
- Generic RC Transmitter.
- LoRa.

[Here](https://github.com/jakkra/RoverController) is my controller supporting both LoRa and websocket control of the Rover.

### Telematics
Rover will automatically send telematics data as soon as someone connects to the websocket server.
TODO is to send this over LoRa to supprt telematics from far away. 

### Driving Modes
There are two ways to drive the rover. Either using a RC controller or from a website hosted on the ESP32.
RC controller is deafult, but as soon as someone connects to the ESP32 AP and a websocket is created, the RC controller will be overriden by the WiFi controller. The website functionality is just as basic as the RC controller right now, but the plan is to add more advanced functionality to the website which is not possible to do on the RC controller.
On the RC Transmitter I use two joysticks for channels 1-4 for, and two switches for channel 5-6.
Joysticks are used both for steering the Rover and for moving each axis of the arm.
When switch1 is:
- LOW: joysticks are used for driving the rover
- MID: Wheels will be set correct angles for the rover to be able to rotate on the spot by running each side of wheels in opposite direction. The direction is descided by the steer joystick.
- HIGH: Arm mode, the two joysticks are used for moving axis 1-4 on the arm. Switch2 is used to switch between moving axis 1-4 and axis 5-6 (the gripper). This will be changed in the future, steering each axis manually is not really a good way, inverse kinematics is TODO.

## Arm
Currently arm is just pretty much mapped to controls on the Controller, this is not really useful, so TODO.

## Hardware
- Uses one DC motor in each wheel
- 2 brushless ESC's one for each side (3 motors)
- Lots of (12, one for each servo) 12V to 5V switching voltage regulators
- 4 servos (MG946R/MG996R) on each corner wheel to steer.
- 6 Servos (MG946R) in the arm for 6-DOF. Poweded by a 3S battery. 
- 2 servos (MG946R/MG996R) for the head.
- 6 Channel RC receiver and transmitter
- 6 Motor shaft adapters https://www.pololu.com/product/1081
- Some Round Metal Servo Arm 25T Disc Horns.
- MCU (ESP32)
- 3S battery
- Standard PVC pipe with inner diameter of 23.40 mm and outer diameter 25 mm.
- Lots of screws and bolts, mostly M3, but some M4's too.
- Two kinds of bearings: 5x 608ZZ and 5x 25mm SKF 6005
- Check the Fusion360 project for more details.
- Cable madness is TODO to fix (temporary fixed by hiding it under the "lid")

## Building
There is no manual, but downloading the CAD model and browsing through it should get you started. Browse through both open and closed issues for help. Feel free to open an issue if you have a question. 

## Images
Videos:
- Showing the boogie mechanism in action [https://imgur.com/4UIsRLl](https://imgur.com/4UIsRLl) 
- Arm [https://imgur.com/KY5ioLb](https://imgur.com/KY5ioLb)
<img src="/.github/2.jpg "/>
<img src="/.github/full.jpg "/>
<img src="/.github/boogie_full.jpg "/>
<img src="/.github/1.jpg "/>
<img src="/.github/4.jpg "/>
<img src="/.github/parts.jpg "/>
<p float="left">
<img src="/.github/arm.jpg" width="420" />
<img src="/.github/back.jpg" width="420" />
</p>
<img src="/.github/full1.jpg "/>
<img src="/.github/cad.png" />
<p float="left">
<img src="/.github/cad_capture.png" width="420" />
<img src="/.github/render.jpg" width="420" />
</p>
<img src="/.github/wifi_controller.jpg" />

### Uploading Controller page to Spiffs
```
pio run -t uploadfs
```

### Misc
```
xtensa-esp32-elf/bin/xtensa-esp32-elf-addr2line.exe -pfiaC -e /c/Users/ijakk_000/Documents/Rover-code/.pio/build/esp-wrover-kit/firmware.elf 0x00000000:0x3ffb5c60 ...
```