# Dual-Axis-Solar-Tracking-System-With-Weather-Monitoring-System-Using-Arduino-UNO
The Dual Axis Solar Tracking System with Weather Monitoring System using Arduino UNO is a practical solution that uses a microcontroller to adjust the angle and orientation of solar panels to maximize their exposure to sunlight, while also monitoring weather conditions to prevent damage.

The process of creating a Dual Axis Solar Tracker Arduino Project using LDR and Servo Motors. With this project, you will learn how to utilize Light Sensitive Sensors, such as LDR, to track the movement of the sun and optimize the positioning of the solar panels, resulting in increased efficiency. We will provide you with a step-by-step guide to help you build this project successfully.



Introduction:

Solar energy has emerged as one of the most promising renewable energy sources in recent years. However, the efficiency of the solar panels is greatly affected by the orientation of the panels with respect to the sun. Solar panels that are stationary throughout the day cannot capture the maximum amount of sunlight, which reduces their efficiency. In order to overcome this problem, a dual-axis solar tracking system can be used. In this blog, we will discuss how to build a dual-axis solar tracking system using Arduino Uno.

Components Required:

Arduino Uno - R3
Servo Motors - SG90 (2)
LDR Sensors (4)
Resistors 10k-ohm (4)
Jumper Wires
Breadboard
Solar Panel
Potentiometer 10k (Not Required)
Working Principle:

The dual-axis solar tracking system is based on the position of the sun in the sky. The system is designed to track the movement of the sun and adjust the orientation of the solar panel accordingly. The system uses LDR (Light Dependent Resistor) sensors to detect the position of the sun in the sky. The LDR sensors sense the intensity of light falling on them and provide an analog output that is proportional to the intensity of light. The Arduino Uno receives this analog output from the LDR sensors and calculates the position of the sun in the sky. The Arduino Uno then controls the servo motors to rotate the solar panel on both the X and Y axis to track the movement of the sun.

Step-by-Step Guide:

Circuit Diagram:
The first step in building the dual-axis solar tracking system is to create the circuit diagram. Connect the LDR sensors to the analog pins of the Arduino Uno. Connect the servo motors to the digital pins of the Arduino Uno. Connect the positive and negative terminals of the solar panel to the breadboard.

Tinkercad Dual Axis Solar Tracker Arduino Simulation file

                                Interfacing Dual Axis Solar Tracker Arduino Project Using LDR & Servo Motors

Go to Tinkercad:- 
https://www.tinkercad.com/things/5jtX4CKdKCz-copy-of-arduino-based-dual-axis-solar-tracker/editel?sharecode=MPgVX3QzLaC2uS1Y_8q-Kpyzi5siWVYdAnn5MFwoa1Y

Connect the 5 volt pin from the Arduino to the Lower horizontal row of the breadboard.
Similarly, connect the GND pin from the Arduino to a second lower horizontal row of the breadboard.
Extend the 5 volt and GND Rows to the upper horizontal rows of the breadboard respectively.
Now connect the power pins of both Vertical and Horizontal servo motors to the 5 volts.
Similarly, Connect the GND pin of the Both Horizontal and Vertical Servo motor to the Ground.
Now, connect the Signal pin of the Vertical Servo Motor to the Digital Pin No. 10 of the Arduino.
Again, connect the Signal pin of the horizontal Servo Motor to the Digital Pin No. 9 of the Arduino.
Connect one terminal of both potentiometers to the Ground and the Other end terminals of both potentiometers to the VCC 5 volt.
Now, connect all LDRs from one terminal to the 5-volt and other terminals to the Ground Through 10k-ohm resistors.
Let’s connect the wiper pin of the potentiometer-1 one to the Analog Pin A4 and A5 to another wiper pin of Potentiometer-2.
Connect bottom left LDR voltage Divider point to A1 pin of Arduino.
Again, Connect Top Left LDR voltage Divider Point to A0 pin of the Arduino.
Similarly, Top Right LDR Voltage Divider Point to the A2 pin.
Finally, Connect the Bottom Right Voltage Divider Point of LDR to the A3 pin of the Arduino.


Code:
Once the circuit diagram is created, the next step is to write the code for the Arduino Uno. The code is responsible for receiving data from the LDR sensors, calculating the position of the sun, and controlling the servo motors to rotate the solar panel. The code can be written in the Arduino IDE, which is a simple and easy-to-use platform for programming the Arduino Uno. The code should be uploaded to the Arduino Uno after it is written.



Programming Arduino for Dual Axis Solar Tracker Project
#Include is used to include a servo header library file.

#include <Servo.h> 
Configuration for Horizontal servo.
The Servo Horizontal is set to 180 degrees.
Servo Horizontal Limit When Signal is High is set to 175 degrees.
Again, Servo Horizontal Limit when the Signal value is low is set to 5 degrees.

Servo horizontal; // horizontal servo
int servoh = 180; 
int servohLimitHigh = 175;
int servohLimitLow = 5;
// 65 degrees MAX
This is the Servo Configuration for Vertical
The Servo Vertical is set to 45 degrees.
Servo Vertical Limit When Signal is High is set to 60 degrees.
Also, Servo Vertical Limit when the Signal value is low is set to 1 degree.

Servo vertical; // vertical servo
int servov = 45; 
int servovLimitHigh = 60;
int servovLimitLow = 1;
LDR Pin Connections
ldrlt is for Top Left
ldrrt is for Top Right
ldrld is for Down Left
ldrrd is for Down Right

// LDR pin connections
// name = analogpin;
int ldrlt = A0; //LDR top left - BOTTOM LEFT <--- BDG
int ldrrt = A3; //LDR top rigt - BOTTOM RIGHT
int ldrld = A1; //LDR down left - TOP LEFT
int ldrrd = A3; //ldr down rigt - TOP RIGHT
On void setup, we have attached the vertical and Horizontal servo signal pin. and the servo rotation for the horizontal is set to 180 degrees. and similarly, the vertical servo is set to 45 degrees. we have also set a delay of 2.5 seconds.

void setup(){
horizontal.attach(9);
vertical.attach(10);
horizontal.write(180);
vertical.write(45);
delay(2500);
}
In the Void loop() Function, we read the value for the analog pin of the Arduino connected to LDRs. and then calculate the average of vertical and horizontal.

void loop() {
int lt = analogRead(ldrlt); // top left
int rt = analogRead(ldrrt); // top right
int ld = analogRead(ldrld); // down left
int rd = analogRead(ldrrd); // down right
int dtime = 10; int tol = 90; // dtime=diffirence time, tol=toleransi
int avt = (lt + rt) / 2; // average value top
int avd = (ld + rd) / 2; // average value down
int avl = (lt + ld) / 2; // average value left
int avr = (rt + rd) / 2; // average value right
int dvert = avt - avd; // check the diffirence of up and down
int dhoriz = avl - avr;// check the diffirence og left and rigt
IF and IF-ELSE statements are defined to loop the program and calculate the average values of the respective LDR’s. Finally, this code helps to change the degree of the servo motor. So that it can be more effective.

if (-1*tol > dvert || dvert > tol) 
 {
 if (avt > avd)
 {
 servov = ++servov;
 if (servov > servovLimitHigh)
 {servov = servovLimitHigh;}
 }
 else if (avt < avd)
 {servov= --servov;
 if (servov < servovLimitLow)
 { servov = servovLimitLow;}
 }
 vertical.write(servov);
 }
if (-1*tol > dhoriz || dhoriz > tol) // check if the diffirence is in the tolerance else change horizontal angle
 {
 if (avl > avr)
 {
 servoh = --servoh;
 if (servoh < servohLimitLow)
 {
 servoh = servohLimitLow;
 }
 }
 else if (avl < avr)
 {
 servoh = ++servoh;
 if (servoh > servohLimitHigh)
 {
 servoh = servohLimitHigh;
 }
 }
 else if (avl = avr)
 {
 delay(5000);
 }
 horizontal.write(servoh);
 }
At last, we have added the delay of (dtime).

delay(dtime);
}
Now, compile the program and upload it to your Arduino board.


The final code for the dual-axis solar tracking system using Arduino Uno is as follows:
C++ Code
#include <Servo.h> 

Servo horizontal; // horizontal servo
int servoh = 180; 
int servohLimitHigh = 175;
int servohLimitLow = 5;
// 65 degrees MAX

Servo vertical; // vertical servo
int servov = 45; 
int servovLimitHigh = 60;
int servovLimitLow = 1;

// LDR pin connections
// name = analogpin;
int ldrlt = A0; //LDR top left - TOP LEFT <--- BDG
int ldrrt = A1; //LDR top rigt - TOP RIGHT
int ldrld = A2; //LDR down left - BOTTOM LEFT
int ldrrd = A3; //ldr down rigt - BOTTOM RIGHT

void setup(){
horizontal.attach(9);
vertical.attach(10);
horizontal.write(180);
vertical.write(45);
delay(2500);
}
void loop() {
int lt = analogRead(ldrlt); // top left
int rt = analogRead(ldrrt); // top right
int ld = analogRead(ldrld); // down left
int rd = analogRead(ldrrd); // down right
int dtime = 10; int tol = 90; // dtime=diffirence time, tol=toleransi
int avt = (lt + rt) / 2; // average value top
int avd = (ld + rd) / 2; // average value down
int avl = (lt + ld) / 2; // average value left
int avr = (rt + rd) / 2; // average value right
int dvert = avt - avd; // check the diffirence of up and down
int dhoriz = avl - avr;// check the diffirence og left and rigt

if (-1*tol > dvert || dvert > tol) 
 {
 if (avt > avd)
 {
 servov = ++servov;
 if (servov > servovLimitHigh)
 {servov = servovLimitHigh;}
 }
 else if (avt < avd)
 {servov= --servov;
 if (servov < servovLimitLow)
 { servov = servovLimitLow;}
 }
 vertical.write(servov);
 }
if (-1*tol > dhoriz || dhoriz > tol) // check if the diffirence is in the tolerance else change horizontal angle
 {
 if (avl > avr)
 {
 servoh = --servoh;
 if (servoh < servohLimitLow)
 {
 servoh = servohLimitLow;
 }
 }
 else if (avl < avr)
 {
 servoh = ++servoh;
 if (servoh > servohLimitHigh)
 {
 servoh = servohLimitHigh;
 }
 }
 else if (avl = avr)
 {
 delay(5000);
 }
 horizontal.write(servoh);
 }
 
 delay(dtime);
}
Testing:
After writing the code, the next step is to test the system. Place the solar panel in direct sunlight and run the code. The servo motors should start rotating, and the solar panel should track the movement of the sun. The orientation of the solar panel should be adjusted continuously to keep it perpendicular to the rays of the sun.

Calibration:
Once the system is tested, the final step is to calibrate the system. This involves adjusting the sensitivity of the LDR sensors and the speed of the servo motors to ensure the maximum efficiency of the solar panel.



Conclusion:


Dual axis solar tracking system is an effective way to increase the efficiency of solar panels. By tracking the movement of the sun, the solar panel can be placed perpendicular to the rays of the sun, resulting in maximum energy output. Building a dual-axis solar tracking system using Arduino Uno is a simple and cost-effective way to take advantage of renewable energy sources. Finally, we have completed Interfacing Dual Axis Solar Tracker Arduino Project Using LDR & Servo Motors. Now, you can use this Project to track the solar panel and increase its efficiency by 40%. We hope you found this project useful! Drop a comment below if you have any doubts or queries. We’ll do our best to answer your questions.
