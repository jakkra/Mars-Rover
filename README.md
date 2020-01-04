# Rover
## CAD model
The whole Fusion 360 project can be found in the CAD folder, download the .f3z file and open it in desktop Fusion 360. There is also a STEP file in the same folder, however I did not export a .step file fo each part.

## Driving modes
There are two ways to drive the rover. Either using a RC controller or from a website hosted on the ESP32.
RC controller is deafult, but as soon as someone connects to the ESP32 AP and a websocket is created, the RC controller will be overriden by the WiFi controller. The website functionality is just as basic as the RC controller right now, but the plan is to add more advanced functionality to the website which is not possible to do on the RC controller.
On the RC Transmitter I use two joysticks for channels 1-4 for, and two switches for channel 5-6.
Joysticks are used both for steering the Rover and for moving each axis of the arm.
When switch1 is:
- LOW: joysticks are used for driving the rover
- MID: Wheels will be set correct angles for the rover to be able to rotate on the spot by running each side of wheels in opposite direction. The direction is descided by the steer joystick.
- HIGH: Arm mode, the two joysticks are used for moving axis 1-4 on the arm. Switch2 is used to switch between moving axis 1-4 and axis 5-6 (the gripper). This will be changed in the future, steering each axis manually is not really a good way, inverse kinematics is TODO.

## Arm
Currently arm is just pretty much mapped to controls on the RC Controller, this is not really useful, so TODO.

## Hardware
- Uses one DC motor in each wheel
- 2 brushless ESC's one for each side (3 motors)
- Lots of (12, one for each servo) 12V to 5V switching voltage regulators
- 4 servos (MG946R) on each corner wheel to steer.
- 6 Servos in the arm for 6-DOF. Poweded by a 3S battery. 
- 2 servos (MG946R) for the head.
- 6 Channel RC receiver and transmitter
- 6 Motor shaft adapters https://www.pololu.com/product/1081
- Some Round Metal Servo Arm 25T Disc Horns.
- MCU (ESP32)
- 3S battery
- Lots of screws and bolts, mostly M3, but some M4's too.
- Two kinds of bearings: 5x 608ZZ and 5x 25mm SKF 6005
- Check the Fusion360 project for more details.
- Cable madness is TODO to fix (temporary fixed by hiding it under the "lid")

## Images
Videos:
- Showing the boogie mechanism in action [https://imgur.com/4UIsRLl](https://imgur.com/4UIsRLl) 
- Arm [https://imgur.com/KY5ioLb](https://imgur.com/KY5ioLb)
<img src="/.github/1.jpg "/>
<img src="/.github/2.jpg "/>
<img src="/.github/full.jpg "/>
<p float="left">
<img src="/.github/arm.jpg" width="420" />
<img src="/.github/back.jpg" width="420" />
</p>
<img src="/.github/4.jpg "/>
<img src="/.github/parts.jpg "/>
<img src="/.github/cad.png" />
<img src="/.github/cad_capture.png" />
<img src="/.github/render.jpg" />
<img src="/.github/wifi_controller.jpg" />

### Uploading Controller page to Spiffs
```
pio run -t uploadfs
```
