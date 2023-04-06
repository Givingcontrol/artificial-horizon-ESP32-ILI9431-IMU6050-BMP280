/* V0
 *  ported to ST7789 TFT - modify user setup in TFT_eSPI
 *  does not fill whole screen - resized V1
 *  
 * V1 
 *  remove most mavlink code
 *  add MPU6050
 *  add MPUlight Libary to read pitch, roll, yaw
 *  resize display to fit 240 x 240 = should be autofitted! 
 *  horizon moves with pitch and roll
 *    compiles and loads
 *    *  has artifacts 
 *    
 * V2 
 *  Change draw horizon function as per post 44  at:
 *  https://forum.arduino.cc/t/artificial-horizon-display-using-a-gyro/402237/43
 *    artifacts removed
 *    
 * V3  
 *  Add BMP280 pressure sensor for altimeter reading
 *  display pressure at boot and in display
 *  
 * V4
 *  Make display auto size based on TFT size
 *  
 *  V5
 *   ILI9341 display : modify user setup in TFT_eSPI 
 *  
 *  
 *  FIX********
 *  MPU angle doesn't match screen angle
 *  
 *  MAVLInk_DroneLights
 *  by Juan Pedro López
 *  This program was developed to connect an Arduino board with a Pixhawk via MAVLink 
 *   with the objective of controlling a group of WS2812B LED lights on board of a quad

// Demo code for artifical horizon display
// Written by Bodmer for a 160 x 128 TFT display
// 15/8/16
// Code templates joined by vierfuffzig feb 2021
// https://github.com/vierfuffzig/MAVLinkEFIS
//************************************************
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MPU6050_light.h>

//I2C ESP32:: SCL = GPIO22 SDA= GPIO21
MPU6050 mpu_light(Wire);
Adafruit_BMP280 bmp;


//MPU variables

int16_t pitch, roll, yaw; //from mpu6050light
int16_t Pitch, Roll, Yaw; //from mpu6050light

//  variables to store bmp280 readings to be sent
float temperature;
float pressure;
float startAltitude;

// Mavlink carry over variables
int16_t heading = 0;
float alt ;
//float volt = 0;
float curr = 0;
int16_t rssi = 0;
//float climb = 0;
float aspd = 0;
int16_t mode = 0;

static float volt = 12.3;
static char voltstr[15];

static float climb = 12.3;
static char climbstr[15];

// String modestr = "MODE";
char modestr[15] = "MODE";






////////////////////////////////////////// TFT Setup ///////////////////////////////////////////////////////////
// Invoke library, pins defined in User_Setup.h  https://github.com/Bodmer/

/* For ESP32 Dev board (only tested with ILI9341 display)
// The hardware SPI can be mapped to any pins

//#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   4  // Reset pin (could connect to RST pin)
//#define TFT_RST  -1  // Set TFT_RST to -1 if display RESET is connected to ESP32 board RST
*/

#include <SPI.h>
#include <TFT_eSPI.h>

const int Y_TFT_height = 320;
const int X_TFT_width = 240;

TFT_eSPI tft = TFT_eSPI();
 // Invoke library, pins defined in User_Setup.h  https://github.com/Bodmer/

#define REDRAW_DELAY 16 // minimum delay in milliseconds between display updates

//V1 #define HOR 250    // Horizon circle outside radius (205 is corner to corner
/*V2*/
#define HOR 400    // Horizon vector line length    

#define BROWN      0x5140 //0x5960
#define SKY_BLUE   0x02B5 //0x0318 //0x039B //0x34BF
#define DARK_RED   0x8000
#define DARK_GREY  0x39C7

#define XC  X_TFT_width/2 // x coord of centre of horizon
#define YC Y_TFT_height/2 // y coord of centre of horizon

#define ANGLE_INC 1 // Angle increment for arc segments, 1 will give finer resolution, 2 or more gives faster rotation

#define DEG2RAD 0.0174532925

int roll_angle = 180; // These must be initialed to 180 so updateHorizon(0); in setup() draws

int last_roll = 0; // the whole horizon graphic
int last_pitch = 0;

int roll_delta = 90;  // This is used to set arc drawing direction, must be set to 90 here


// Variables for test only
//int test_angle = 0;
//int delta = ANGLE_INC;

unsigned long redrawTime = 0;



////////////////////////////////////////////// end of tft setup ////////////////////////////////////////////////////////


void setup() {
  
  Serial.begin(115200);
Wire.begin();
   mpu_light.begin();
   bmp.begin(0x76);

//////////Start up calibration///////////////////////////////////
tft.begin();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED);
  tft.setTextSize(2);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("Calculating", 120, 80, 1);
  tft.drawString("Gyro Offsets", 120, 120, 1);
  mpu_light.calcGyroOffsets(); 
   delay(2000);

 //BMP readings
  get_BMP_Readings();
  startAltitude = bmp.readAltitude(1013.25);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED);
  tft.setTextSize(2);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("temperature", 120, 5, 2);
  tft.drawNumber(temperature, 120, 50, 2);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("pressure", 120, 95, 2);
  tft.drawNumber(pressure, 120, 140, 2);
   tft.drawString("Alt", 120, 160, 2);
  tft.drawNumber(startAltitude, 120, 205, 2);
   delay(2000);
   
  

  
   
///////////////// Horizon TFT setup ///////////////////////////////////////////////////
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);   
  
/*
//240 x 240 display
  tft.fillRect(0,  0, 240, 120, SKY_BLUE);
  tft.fillRect(0, 120, 240, 120, BROWN);
*/
  //Auto display horizon split
  tft.fillRect(0,  0, X_TFT_width, YC, SKY_BLUE);
  tft.fillRect(0, YC, X_TFT_width, Y_TFT_height, BROWN);

  // Draw the horizon graphic
  drawHorizon(0, 0);
  drawInfo();
  delay(2000); // Wait to permit visual check
  
}

void loop() {
  
  
  get_MPU_Angle_Readings();
  get_BMP_Readings();
  


  

////////////////////////////////////// TFT display //////////////////////////

  if (millis() > redrawTime) {
    
    redrawTime = millis() + REDRAW_DELAY;
    updateHorizon(roll, pitch);

  }
   
}

//////////////////////////////////////MPU functions
void get_MPU_Angle_Readings(){
//mpu6050light  
   mpu_light.update();
  Pitch =(mpu_light.getAngleX());
  pitch = Pitch;
  //alt = Pitch;  // getting values to display
  roll =(mpu_light.getAngleY());
  climb = roll;
  Yaw = (mpu_light.getAngleZ());
}

void get_BMP_Readings(){
  temperature = bmp.readTemperature();
  pressure = (bmp.readPressure() / 100.0F);
  
  alt =(bmp.readAltitude(1013.25) - startAltitude) ;
}

// #########################################################################
// Update the horizon with a new angle (angle in range -180 to +180)
// #########################################################################
void updateHorizon(int roll, int pitch)
{
  bool draw = 1;
  int delta_pitch = 0;
  int pitch_error = 0;
  int delta_roll  = 0;
  while ((last_pitch != pitch) || (last_roll != roll))
  {
    delta_pitch = 0;
    delta_roll  = 0;

    if (last_pitch < pitch) {
      delta_pitch = 1;
      pitch_error = pitch - last_pitch;
    }
    
    if (last_pitch > pitch) {
      delta_pitch = -1;
      pitch_error = last_pitch - pitch;
    }
    
    if (last_roll < roll) delta_roll  = 1;
    if (last_roll > roll) delta_roll  = -1;
    
    if (delta_roll == 0) {
      if (pitch_error > 1) delta_pitch *= 2;
    }
    
    drawHorizon(last_roll + delta_roll, last_pitch + delta_pitch);
    drawInfo();
  }
}

//V2 updated draw horizon
// #########################################################################
// Draw the horizon with a new roll (angle in range -180 to +180)
// #########################################################################

void drawHorizon(int roll, int pitch)
{
  // Calculate coordinates for line start
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);

  int16_t x0 = sx * HOR;
  int16_t y0 = sy * HOR;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn  = 0;
  int16_t ydn = 0;

  if (roll > 45 && roll <  135) {
    xd = -1;
    yd =  0;
  }
  if (roll >=  135)             {
    xd =  0;
    yd = -1;
  }
  if (roll < -45 && roll > -135) {
    xd =  1;
    yd =  0;
  }
  if (roll <= -135)             {
    xd =  0;
    yd = -1;
  }

  if ((roll != last_roll) || (pitch != last_pitch))
  {
    xdn = 6 * xd;
    ydn = 6 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
    xdn = 5 * xd;
    ydn = 5 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
    xdn = 4 * xd;
    ydn = 4 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
    
    xdn = 3 * xd;
    ydn = 3 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
  }
  xdn = 2 * xd;
  ydn = 2 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);

  tft.drawLine(XC - x0 - xd, YC - y0 - yd - pitch, XC + x0 - xd, YC + y0 - yd - pitch, SKY_BLUE);
  tft.drawLine(XC - x0 + xd, YC - y0 + yd - pitch, XC + x0 + xd, YC + y0 + yd - pitch, BROWN);

  tft.drawLine(XC - x0, YC - y0 - pitch,   XC + x0, YC + y0 - pitch,   TFT_WHITE);

  last_roll = roll;
  last_pitch = pitch;

}

// #########################################################################
// Draw the information
// #########################################################################

void drawInfo(void)
{
  // Update things near middle of screen first (most likely to get obscured)

  // Level wings graphic
  tft.fillRect(XC - 2, YC - 2, 5, 5, TFT_RED);
  tft.drawFastHLine(XC - 55,   YC, 50, TFT_RED);
  tft.drawFastHLine(XC + 55 - 50, YC, 50, TFT_RED);
  tft.drawFastVLine(XC - 45 + 40, YC, 5, TFT_RED);
  tft.drawFastVLine(XC + 45 - 40, YC, 5, TFT_RED);

  tft.drawFastHLine(XC - 60,   YC - 40, 120, TFT_WHITE); //20 deg horizontal
  tft.drawFastHLine(XC -  45,   YC - 30, 90, TFT_WHITE); //15 deg horizontal
  tft.drawFastHLine(XC - 60,   YC - 20, 120, TFT_WHITE); //10 deg horizontal
  tft.drawFastHLine(XC -  45,   YC - 10, 90, TFT_WHITE); //5 deg horizontal

  tft.drawFastHLine(XC -  45,   YC + 10, 90, TFT_WHITE); //-5 deg horizontal
  tft.drawFastHLine(XC - 60,   YC + 20, 120, TFT_WHITE); //-10 deg horizontal
  tft.drawFastHLine(XC -  45,   YC + 30, 90, TFT_WHITE); //-15 deg horizontal
  tft.drawFastHLine(XC - 60,   YC + 40, 120, TFT_WHITE); //-20 deg horizontal

// Pitch numbers  upper 10....lower 10 left and right
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(XC - 60 - 20, YC - 20 - 3); //left +10
  tft.print("10"); //left +10
  tft.setCursor(XC + 60 + 3 , YC - 20 - 3); // Right +10
  tft.print("10"); //left -10
  tft.setCursor(XC - 60 - 20, YC + 20 - 3); //left -10
  tft.print("10");//Right +10
  tft.setCursor(XC + 60 + 3, YC + 20 - 3);//Right -10
  tft.print("10"); //Right -10

// Pitch numbers  lower 20....upper 20 left and right
  tft.setCursor(XC - 60 - 20, YC - 40 - 3);//+left 20
  tft.print("20"); //left 20
  tft.setCursor(XC + 60 + 3, YC - 40 - 3); // +Right 20
  tft.print("20");//left 20
  tft.setCursor(XC - 60 - 20, YC + 40 - 3);//-left 20
  tft.print("20");//Right 20
  tft.setCursor(XC + 60 + 3, YC + 40 - 3); //-Right 20
  tft.print("20");//Right 20


/*****flight data*******
 * 
 *   int16_t heading = 0;
 *   float alt = 0;
 *   float volt = 0;
 *   float curr = 0;
 *   int16_t rssi = 0;
 *   float climb = 0;
 *   float aspd = 0;
 *   int16_t mode = 0;
 */

  // Display flight data
  // Top Data
  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);
  
  tft.setTextPadding(88); // Flightmode requires full line padding
  tft.drawString(modestr, 120, 1, 1);
  
  tft.setTextPadding(24); // reset to value only padding
  tft.drawNumber(alt, 129, 10, 1);
 // tft.drawString(climbstr, 74, 19, 1);
  //tft.drawString("12.3", 74, 19, 1);
   tft.drawNumber(climb, 129, 19, 1);

  // Bottom data
  tft.setTextColor(TFT_YELLOW, BROWN); // Text with background
  tft.setTextDatum(MC_DATUM);            // Centre middle justified
  tft.setTextPadding(24);                // Padding width to wipe previous number
 
  tft.drawNumber(heading, 120, 180, 1);
  tft.drawNumber(rssi, 120, 200, 1);
  tft.drawString(voltstr, 120, 220, 1);
  //tft.drawNumber(curr, 74, 151, 1);
  //tft.drawNumber(alt, 64, 151, 1);

  // Draw fixed text
  tft.setTextColor(TFT_YELLOW);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  
  tft.drawString("Alt", 70, (Y_TFT_height- Y_TFT_height+20), 1);
  tft.drawString("m", 160, (Y_TFT_height- Y_TFT_height+20), 1);
  tft.drawString("Climb", 70, (Y_TFT_height- Y_TFT_height+40), 1);
  tft.drawString("m/s", 160, (Y_TFT_height- Y_TFT_height+40), 1);
  tft.drawString("Hdg", 80, (Y_TFT_height-60), 1);
  tft.drawString("deg", 160, (Y_TFT_height-60), 1);
  tft.drawString("Rssi", 80, (Y_TFT_height-40), 1);
  tft.drawString("%", 160, (Y_TFT_height-40), 1);
  tft.drawString("Bat", 80, (Y_TFT_height-20), 1);
  tft.drawString("V", 160, (Y_TFT_height-20), 1);

// Draw Bug holders
  //airspeed
  tft.fillRect(0,95,40,50,TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextDatum(TC_DATUM);
  tft.setTextPadding(24); // reset to value only padding
  tft.drawNumber(climb, 15, 112, 2);
  // alt
  tft.setTextColor(TFT_YELLOW);
  tft.setTextDatum(TC_DATUM);
  tft.fillRect(200,95,40,50,TFT_BLACK);
  tft.setTextPadding(24); // reset to value only padding
  tft.drawFloat(alt, 25, 110, 2);
  // heading
  tft.fillRect(90,0,60,50,TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  tft.drawFloat(alt, 2 , 100, 10, 2);
  Serial.println(alt);

// Draw bug text

  
}
