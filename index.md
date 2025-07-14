# Real Time Planet Tracker
My project is a real time planet tracker. What this project does is it tracks the planets coordinates using azimuth and altitude and points to where the planet is in the sky with a servo with a laser attached. This can go through all planets execpt earth and you can control what planet is calculates with a button. The main challenges I faced was dealing with the Servo tangling due to over rotations, Dealing with the Azimuth and Altitude Math, and dealing with a not working IMU which I decided to scrap. 
| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Aaditya P | California High School| Aerospace Engineering | Incoming Senior

<img src = "AadityaP.heic.jpg" width = "450" height = "600">
  
<!--# Final Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/K05nV2iawsE?si=kE9Uvx09qNqWfP08" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

For your final milestone, explain the outcome of your project. Key details to include are:
- What you've accomplished since your previous milestone
- What your biggest challenges and triumphs were at BSE
- A summary of key topics you learned about
- What you hope to learn in the future after everything you've learned at BSE

-->

# Second Milestone



<iframe width="560" height="315" src="https://www.youtube.com/embed/K05nV2iawsE?si=kE9Uvx09qNqWfP08" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

# Summary 

My second milestone in my project is making sure the calculations for the Azimuth and Altitude were working well. So to test that they work I use the planet Mars and told the code to print out the azimuth, altitude, right ascention, and declination. This is to basically find the coordinates of where the planet is at and it works well. For my future milestones I plan on adding all the other planets in the solar system and cycle them using a button switcher.  

# Challenges 

The main challenge I faced with this part was the servo over rotating and chocking itself which messes up the connections with the breadboard and ardino. To fix this I just shorted the rotation to only 360 degrees and this worked because my servo motor stopped chocking itself. Another problem I had was figuring out what I was going to do with the IMU since it was not working at all . What I decided to do is to scrap the IMU and just angle the Servos North for the most accurate results. 

# First Milestone

<iframe width="560" height="315" src="https://www.youtube.com/embed/h4RonOS_DbQ?si=PFpO-KHJd071FopQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

# Summary

My first milestone in my project is making sure all of the connections in the breadboard and Arduino work. This includes connecting the servo motors, GPS module, and IMU. To make this work, I ran a simple code to test everything out. For my future milestones, I am planning on making a better code so it can track the planets, and also improve the servo motor and attach the laser to it. 

# Challenges

Some challenges I faced with this first milestone include making sure all of the connections were correct and what I did to help me through this problem was cross refernecing outside sources to make sure it was correct. Another Problem I had was building the servo motor since it was very difficult due to the small screws and it would break very easily. What I did to overcome this problem was just going through the steps very slowly so I ensure there is no mistakes. 

# Schematics 
<img src = "1a30a69f-ffd0-46b4-83b8-59b102e71529 (1).JPG" width = "900" height = "600">

<a href="https://paulplusx.wordpress.com/2016/03/03/rtpts_hw/">shubhampaul tinkercad</a>



# Code

```python
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <TinyGPSPlus.h>

// Servo driver setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels
const uint8_t panChannel  = 0;
const uint8_t tiltChannel = 1;

// Safe microsecond limits for standard servos
const uint16_t panMinUs = 1300;
const uint16_t panMaxUs = 1700;
const uint16_t tiltMinUs = 1200;
const uint16_t tiltMaxUs = 1800;

// GPS setup (pins 15/16 on Arduino Mega)
TinyGPSPlus gps;

// Planet data (Earth excluded)
const char* planetNames[] = {
  "Mercury", "Venus", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune", "Pluto"
};

const double azimuths[] = {
  167.4, 271.6, 124.6, 252.4, 293.3, 275.9, 294.2, 13.1
};

const double altitudes[] = {
  68.5, 29.6, 47.1, 58.8, -29.6, 26.1, -29.0, -75.6
};

const int planetCount = sizeof(planetNames) / sizeof(planetNames[0]);
int planetIndex = 0;

// Button pin
const int buttonPin = 44;
bool buttonPressed = false;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  pinMode(buttonPin, INPUT); // No internal pull-up/pull-down, use your 5V/GND logic

  Serial.println(" Planet Tracker Initialized");
  printPlanetInfo(planetIndex);
  moveServos(azimuths[planetIndex], altitudes[planetIndex]);
}

void loop() {
  while (Serial3.available()) {
    gps.encode(Serial3.read());
  }

  // Button press detection
  if (digitalRead(buttonPin) == HIGH && !buttonPressed) {
    planetIndex = (planetIndex + 1) % planetCount;
    printPlanetInfo(planetIndex);
    moveServos(azimuths[planetIndex], altitudes[planetIndex]);
    buttonPressed = true;
    delay(250); // debounce
  }

  if (digitalRead(buttonPin) == LOW) {
    buttonPressed = false;
  }
}

// Move the servos based on azimuth and altitude
void moveServos(double az, double alt) {
  uint16_t panPWM = mapAngleToPWM(az, 0, 360, panMinUs, panMaxUs);
  uint16_t tiltPWM = mapAngleToPWM(constrain(alt, 0, 90), 0, 90, tiltMinUs, tiltMaxUs);

  panPWM = constrain(panPWM, panMinUs, panMaxUs);
  tiltPWM = constrain(tiltPWM, tiltMinUs, tiltMaxUs);

  pwm.writeMicroseconds(panChannel, panPWM);
  pwm.writeMicroseconds(tiltChannel, tiltPWM);
}

// Convert an angle to PWM signal
uint16_t mapAngleToPWM(double angle, double minAngle, double maxAngle, uint16_t minPWM, uint16_t maxPWM) {
  return minPWM + (angle - minAngle) * (maxPWM - minPWM) / (maxAngle - minAngle);
}

// Print the selected planet info
void printPlanetInfo(int index) {
  Serial.print("Now Tracking: ");
  Serial.println(planetNames[index]);
  Serial.print("Azimuth: ");
  Serial.print(azimuths[index]);
  Serial.println("°");
  Serial.print("Altitude: ");
  Serial.print(altitudes[index]);
  Serial.println("°\n");
}
``` 

# Bill of Materials

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| Arduino Mega 2560 REV3| It is used as the brains of the project | $48.99 | <a href="https://www.microcenter.com/product/621387/arduino-mega-2560-rev3-256kb-(8kb-after-bootloader)-flash-memory/"> Link </a> |
| Neo-6 GPS transmission |This item is used to detect where each planet is| $27.25 | <a href="https://www.u-blox.com/en/product/neo-6-series/"> Link </a> |
| Pan- tilt Mechanism with Servos | What the item is used for | $42.70 | <a href="https://www.mouser.com/ProductDetail/Pimoroni/PIM183?qs=lc2O%252BfHJPVaXow9v4C2FMg%3D%3D&mgh=1&srsltid=AfmBOoogak1-TGBJgu9YiKBZ7QnChSs9LWGuSNQrc7gfcI5SXRs88YEiMP8&gQT=1/"> Link </a> |
| Green Laser Pointer | Show where the planet is in a closed room | $25.99 | <a href="https://www.amazon.com/HITEKK-Pointer-Rechargeable-Tactical-Carrying/dp/B0DJS15VWP?gQT=1/"> Link </a> |
| Power Distrubution Board | Used for Power  | $16.20 | <a href="https://www.keyestudio.com/products/keyestudio-4-channel-l298p-motor-drives-shield-v10-for-arduino-robot/"> Link </a> |



<!--# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/) **
-->
# RGB Sliders 


<iframe width="560" height="315" src="https://www.youtube.com/embed/I4OzfxXsNjA?si=r3gweBtMdo1lx9lo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

My starter Project is RGB Sliders and this shows 3 main colors ( Red, Blue, Green) through sliders. To use this you must have it connected with USB-A, once connected you can slide up the sliders and this will activate the colors with the respective slider you push. The sliders are potentiometers so the more you slide up the more power is outputted.  You can also mix colors by pushing mutliple sliders and have a white light with all 3 sliders pushed. Some challenged I faced was avoiding short circuits because the wires were so close to each other making soldering pretty difficult. 

<img src = "IMG_7674.HEIC.jpg" width = "450" height = "600">
