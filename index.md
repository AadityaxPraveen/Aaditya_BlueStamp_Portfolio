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

<img src = "IMG_7745.jpg" width = "900" height = "600">

# Code

```python
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

/* ─── Hardware constants ───────────────────────────────────────────── */
Adafruit_PWMServoDriver pwm;           // PCA9685 @ 0x40
TinyGPSPlus            gps;           // GPS module on Serial2

const uint8_t  PAN_CH   = 0;
const uint8_t  TILT_CH  = 1;
const uint16_t PAN_MIN  = 1300, PAN_MAX  = 1700;   // µs
const uint16_t TILT_MIN = 1200, TILT_MAX = 1800;   // µs
const uint8_t  BTN_PIN  = 44;                      // active-LOW button

/* ─── J2000 orbital elements (VSOP87) ───────────────────────────────── */
struct Elem { double a,e,I,L,P,O,n; };             // a AU, angles deg, n deg/day
static const Elem PLANET[9] = {
/*idx  a         e         I°        L°         P°         Ω°        n°/d */
  {0.38709893,0.20563069, 7.00487, 252.25084,  77.45645,  48.33167, 4.09233445}, // 0 Mercury
  {0.72333199,0.00677323, 3.39471, 181.97973, 131.60247,  76.67069, 1.60213603}, // 1 Venus
  {1.00000011,0.01671022,-0.00015, 100.46435, 102.94719, -11.26064, 0.98560899}, // 2 Earth
  {1.52366231,0.09341233, 1.85061, 355.45332, 336.04084,  49.57854, 0.52403979}, // 3 Mars
  {5.20336301,0.04839266, 1.30530,  34.40438,  14.75385, 100.55615, 0.08308677}, // 4 Jupiter
  {9.53707032,0.05415060, 2.48446,  49.94432,  92.43194, 113.71504, 0.03344414}, // 5 Saturn
  {19.19126393,0.04716771, 0.76986, 313.23218, 170.96424,  74.22988, 0.01172834},// 6 Uranus
  {30.06896348,0.00858587, 1.76917, 304.88003,  44.97135, 131.72169, 0.00602076},// 7 Neptune
  {39.48168677,0.24880766,17.14175, 238.92881, 224.06676, 110.30347, 0.00395800} // 8 Pluto
};

/* display order (skip Earth) */
const uint8_t TRACK_IDX[8]  = {0,1,3,4,5,6,7,8};
const char*   TRACK_NAME[8] = {
  "Mercury","Venus","Mars","Jupiter",
  "Saturn","Uranus","Neptune","Pluto"
};

/* ─── Math helpers ─────────────────────────────────────────── */
#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)
static inline double d2r(double d){ return d*D2R; }
static inline double r2d(double r){ return r*R2D; }
static inline double wrap360(double d){ d=fmod(d,360.0); return d<0?d+360.0:d; }

/* Julian Day & GMST */
static double julianDayUTC(int y,int m,int D,double hr){
  if(m<=2){ y--; m+=12; }
  int A=y/100, B=2-A+A/4;
  long J=(long)(365.25*(y+4716)) + (long)(30.6001*(m+1)) + D + B - 1524;
  return J + hr/24.0;
}
static double gmstDeg(double jd){
  double T=(jd-2451545.0)/36525.0;
  double g=280.46061837 + 360.98564736629*(jd-2451545.0)
          + 0.000387933*T*T - T*T*T/38710000.0;
  return wrap360(g);
}

/* Kepler chain */
static double meanAnom(const Elem&e,double d){ return wrap360(e.n*d + (e.L - e.P)); }
static double trueAnom(double Mdeg,double e){
  double M=d2r(Mdeg);
  double v=M + (2*e - pow(e,3)/4)*sin(M)
              + 1.25*e*e*sin(2*M)
              + (13.0/12.0)*pow(e,3)*sin(3*M);
  return wrap360(r2d(v));
}
static double radiusAU(const Elem&e,double vdeg){
  return e.a*(1-e.e*e.e)/(1+e.e*cos(d2r(vdeg)));
}
static void heliocXYZ(const Elem&e,double vdeg,double r,
                      double &x,double &y,double &z){
  double O=d2r(e.O), I=d2r(e.I), w=d2r(e.P-e.O), v=d2r(vdeg);
  double cosO=cos(O), sinO=sin(O), cosI=cos(I), sinI=sin(I);
  double cosVW=cos(v+w), sinVW=sin(v+w);
  x = r*(cosO*cosVW - sinO*sinVW*cosI);
  y = r*(sinO*cosVW + cosO*sinVW*cosI);
  z = r*(sinVW*sinI);
}
static void eclToEqu(double x,double y,double z,double &X,double &Y,double &Z){
  const double eps=d2r(23.43928);
  X=x; Y=y*cos(eps)-z*sin(eps); Z=y*sin(eps)+z*cos(eps);
}
static void raDec(double X,double Y,double Z,double &ra,double &dec){
  ra  = wrap360(r2d(atan2(Y,X)));
  dec = r2d(atan2(Z, sqrt(X*X + Y*Y)));
}
static void raDecToAzAlt(double ra,double dec,double jd,double lat,double lon,
                         double &az,double &alt){
  double lst = wrap360(gmstDeg(jd) - lon);   // west-negative longitude
  double ha  = wrap360(lst - ra);
  double haR=d2r(ha), decR=d2r(dec), latR=d2r(lat);
  alt = r2d(asin( sin(decR)*sin(latR) + cos(decR)*cos(latR)*cos(haR) ));
  double sinAz = sin(haR);
  double cosAz = cos(haR)*sin(latR) - tan(decR)*cos(latR);
  az  = r2d(atan2(sinAz, cosAz)); if(az<0) az+=360.0;
}
static void computePlanet(uint8_t elemIdx,double jd,double lat,double lon,
                          double &ra,double &dec,double &az,double &alt){
  const Elem &p = PLANET[elemIdx];    // target planet
  const Elem &e = PLANET[2];          // Earth
  double d = jd - 2451545.0;

  /* planet heliocentric */
  double Mp=meanAnom(p,d), vp=trueAnom(Mp,p.e), rp=radiusAU(p,vp);
  double xp,yp,zp; heliocXYZ(p,vp,rp,xp,yp,zp);

  /* Earth heliocentric */
  double Me=meanAnom(e,d), ve=trueAnom(Me,e.e), re=radiusAU(e,ve);
  double xe,ye,ze; heliocXYZ(e,ve,re,xe,ye,ze);

  /* geocentric vector → equatorial → horizon */
  double X=xp-xe, Y=yp-ye, Z=zp-ze, Xq,Yq,Zq; eclToEqu(X,Y,Z,Xq,Yq,Zq);
  raDec(Xq,Yq,Zq,ra,dec);
  raDecToAzAlt(ra,dec,jd,lat,lon,az,alt);
}

/* Servo helpers */
static uint16_t pwmMap(double v,double in0,double in1,
                       uint16_t out0,uint16_t out1){
  return (uint16_t)(out0 + (v-in0)*(out1-out0)/(in1-in0));
}
static void moveServos(double az,double alt){
  alt = constrain(alt, 0.0, 90.0);
  uint16_t panPWM  = pwmMap(az , 0, 360, PAN_MIN , PAN_MAX );
  uint16_t tiltPWM = pwmMap(alt, 0,  90, TILT_MIN, TILT_MAX );
  panPWM  = constrain(panPWM , PAN_MIN , PAN_MAX );
  tiltPWM = constrain(tiltPWM, TILT_MIN, TILT_MAX );
  pwm.writeMicroseconds(PAN_CH , panPWM );
  pwm.writeMicroseconds(TILT_CH, tiltPWM);
}

/* ─── Globals ─────────────────────────────────────────────── */
int  curIdx      = 0;      // index in TRACK_… arrays
bool btnLatched  = false;
int  lastPrinted = -1;

/* ─── Arduino setup / loop ───────────────────────────────── */
void setup(){
  Serial.begin(9600);
  Serial2.begin(9600);          // GPS on HW Serial2
  Wire.begin(); pwm.begin(); pwm.setPWMFreq(50);
  pinMode(BTN_PIN, INPUT_PULLUP);
  Serial.println(F("PlanetTracker – real-time"));
}

void loop()
{
  /* feed GPS parser */
  while (Serial2.available())
    gps.encode(Serial2.read());

  /* UTC date & time */
  int y, m, d, h, mn, s;  double hr;
  if (gps.date.isValid() && gps.time.isValid()){
    y = gps.date.year();  m = gps.date.month(); d = gps.date.day();
    h = gps.time.hour();  mn = gps.time.minute(); s = gps.time.second();
                             // compile-time fallback
                             } else {                              // force exact UTC for table comparison
    y  = 2025;   // YYYY
    m  = 7;      // MM
    d  = 16;     // DD
    h  = 21;     // HH  (UTC)
    mn = 3;      // MM
    s  = 0;      // SS
}

  }
  hr = h + mn/60.0 + s/3600.0;
  double jd = julianDayUTC(y, m, d, hr);

  /* observer site */
  double lat = gps.location.isValid() ? gps.location.lat() : 37.3142;
  double lon = gps.location.isValid() ? gps.location.lng() : -121.9686; // west-neg

  /* button debounce / cycle planets */
  if (!digitalRead(BTN_PIN) && !btnLatched){
    curIdx = (curIdx + 1) % 8;
    btnLatched = true;
    delay(250);
  }
  if (digitalRead(BTN_PIN))
    btnLatched = false;

  /* compute pointing & drive servos */
  uint8_t elemIdx = TRACK_IDX[curIdx];
  double ra, dec, az, alt;
  computePlanet(elemIdx, jd, lat, lon, ra, dec, az, alt);
  moveServos(az, alt);

  /* print once per planet change */
  if (curIdx != lastPrinted){
    lastPrinted = curIdx;
    Serial.println(F("--------------------------------"));
    Serial.print  (F("Planet: ")); Serial.println(TRACK_NAME[curIdx]);
    Serial.print  (F("RA   (deg): ")); Serial.println(ra , 4);
    Serial.print  (F("Dec  (deg): ")); Serial.println(dec, 4);
    Serial.print  (F("Az   (deg): ")); Serial.println(az , 4);
    Serial.print  (F("Alt  (deg): ")); Serial.println(alt, 4);
    Serial.println();
  }
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

<img src = "Screenshot 2025-07-14 at 1.26.22 PM (1).jpeg" width = "55%">
