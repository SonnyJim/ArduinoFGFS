#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD
#include <Arduino.h>
#include <U8x8lib.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#include <Servo.h>
#include <FastLED.h>

#define NUM_LEDS 2
#define RGB_DATA_PIN 12
#define SERVO_PIN 13

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
//U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8x8(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

CRGB leds[NUM_LEDS];
Servo compass;

int battery_bus_sw;
int avionics_sw;
unsigned long altitude;
int adf1;
int adf1_standby;

int indicated_as;

int ap_passive_mode;
unsigned long ap_target_altitude;
enum ap_altitude_mode_t {AP_ALT_OFF, AP_ALT_PITCH, AP_ALT_HOLD};
enum ap_altitude_mode_t ap_altitude_mode;

enum ap_heading_mode_t {AP_HDG_OFF, AP_HDG_WINGLEVELER, AP_HDG_HOLD, AP_HDG_NAV1, AP_HDG_NAV2, AP_HDG_TRUE};
enum ap_heading_mode_t ap_heading_mode;
int ap_hdg_bug;
float fuel_lbs_left, fuel_lbs_right;
float compass_hdg;


int rpm;

unsigned long tx_millis;
unsigned long lcd_millis;
unsigned long servo_millis;

#define LCD_DELAY 100
#define TX_DELAY 33
#define SERVO_DELAY 15
#define MAGIC_WORD "Pooslice"

#define LED_AP_PASSIVE_MODE 2
#define LED_AP_ALT_MODE 3
#define LED_AP_HDG_MODE 4

int servo_pos;
int servo_direction;



void setup() {
  Serial.begin(9600);
  
  lcd.init();
  lcd.backlight();
  
  u8x8.begin();
  u8x8.setPowerSave(0);  
  
  compass.attach(SERVO_PIN);
  servo_millis = millis();
  compass.write(50);
  FastLED.addLeds<NEOPIXEL, RGB_DATA_PIN>(leds, NUM_LEDS); 

  
  vars_clear ();
  pinMode(LED_AP_PASSIVE_MODE, OUTPUT);
  digitalWrite(LED_AP_PASSIVE_MODE, LOW);
  pinMode(LED_AP_ALT_MODE, OUTPUT);
  digitalWrite(LED_AP_ALT_MODE, LOW);
  pinMode(LED_AP_HDG_MODE, OUTPUT);
  digitalWrite(LED_AP_HDG_MODE, LOW);
}

void loop() {
    leds[0] = CRGB::Blue;
    leds[1] = CRGB::Green;
    FastLED.show();
    vars_update();  
    lcd_update();
    led_update ();
    compass_update ();
/*
    if (millis() > (tx_millis + TX_DELAY))
    {
      tx_millis = millis();
    Serial.print(int(99));
    Serial.print(",");
    Serial.print(int(99));
    Serial.print("\n");
    }
*/
}

void compass_update ()
{
  if (millis() < (servo_millis + SERVO_DELAY))
    return;

  servo_millis = millis();

  servo_pos += servo_direction;

  if (servo_pos > 180)
    servo_direction = -1;
  else if (servo_pos == 0)
    servo_direction = 1;

  compass.write(servo_pos);
}

void vars_clear ()
{
  battery_bus_sw = 0;
  avionics_sw = 0;
  altitude = 0;
  adf1 = 0;
  adf1_standby = 0;
  indicated_as = 0;
  ap_target_altitude = 0;
  ap_altitude_mode = AP_ALT_OFF;
  rpm = 0;

  servo_pos = 0;
  servo_direction = 1;
}

void vars_update()
{
    String buf;
    //char magic_buf[strlen(MAGIC_WORD)];
    
    if (Serial.available())
    {
      /*
    Serial.readBytes(magic_buf, strlen(MAGIC_WORD)+1); //Pooslice is 8 byte
    if (strcmp(magic_buf, MAGIC_WORD) != 0)
      return;
      */
    buf = Serial.readStringUntil(',');
    battery_bus_sw = buf.toInt();
    
    buf = Serial.readStringUntil(',');
    avionics_sw = buf.toInt();
    
    buf = Serial.readStringUntil(',');
    altitude = buf.toInt();
//RADIOS
    buf = Serial.readStringUntil(',');
    adf1 = buf.toInt();
    buf = Serial.readStringUntil(',');
    adf1_standby = buf.toInt();

    buf = Serial.readStringUntil(',');
    indicated_as = buf.toInt();

//AUTOPILOT
    buf = Serial.readStringUntil(',');
    ap_passive_mode = buf.toInt();
    buf = Serial.readStringUntil(',');
    ap_target_altitude = buf.toInt();
    
    buf = Serial.readStringUntil(',');
    if (buf == "altitude-hold")
      ap_altitude_mode = AP_ALT_HOLD;
    else if (buf =="pitch-hold")
      ap_altitude_mode = AP_ALT_PITCH;
    else
      ap_altitude_mode = AP_ALT_OFF;
    
    buf = Serial.readStringUntil(',');
    if (buf == "wing-leveler")
      ap_heading_mode = AP_HDG_WINGLEVELER;
    else if (buf == "dg-heading-hold")
      ap_heading_mode = AP_HDG_HOLD;
    else if (buf == "nav1-hold")
      ap_heading_mode = AP_HDG_NAV1;  
    else if (buf == "nav2-hold")
      ap_heading_mode = AP_HDG_NAV2;  
    else if (buf == "true-heading-hold")
      ap_heading_mode = AP_HDG_TRUE;
    else
      ap_heading_mode = AP_HDG_OFF;
      
    buf = Serial.readStringUntil(',');
    ap_hdg_bug = buf.toInt();
    
//FUEL
    buf = Serial.readStringUntil(',');
    fuel_lbs_left = buf.toFloat();
    buf = Serial.readStringUntil(',');
    fuel_lbs_right = buf.toFloat();

    buf = Serial.readStringUntil(',');
    compass_hdg = buf.toFloat();
//ENGINES            
    buf = Serial.readStringUntil('\n');
    rpm = buf.toInt();
    }

}
void u8x8_draw_fuel()
{
  u8x8.clearBuffer();
  u8x8.setFontDirection(1);

  //Draw the labels
  u8x8.setFont(u8g2_font_ncenB08_tr);
  int x = 110;
  int y = 12;
  u8x8.drawStr(x, y, "F");
  u8x8.drawStr(x - 12, y, "U");
  u8x8.drawStr(x - 24, y, "E");
  u8x8.drawStr(x - 36, y, "L");
  u8x8.drawStr(0,0,"L");
  u8x8.drawStr(0,32-7,"R");

  //draw the bars
  u8x8.drawBox(16,0,map(fuel_lbs_left,0,3000,0,128),8);
  u8x8.drawBox(16,24,map(fuel_lbs_right,0,3000,0,128),8);
  //Draw the ticks
  u8x8.setDrawColor(2);
  for (int i = 1;i < 10;i++)
   {
    if (i % 2 == 0 )
    {
      u8x8.drawLine(16 + (i*11), 0, 16 + (i*11), 4);
      u8x8.drawLine(16 + (i*11), 32, 16 + (i*11), 28);
    }
    else
    {
      u8x8.drawLine(16 + (i*11), 0, 16 + (i*11), 2);
      u8x8.drawLine(16 + (i*11), 32, 16 + (i*11), 30);
    }
   }
    
  u8x8.sendBuffer();
}

void u8x8_draw_compass()
{
    int j,y;
    u8x8.clearBuffer();
    u8x8.setFont(u8g2_font_ncenB18_tr);
    if (compass_hdg >= 64)
      j = compass_hdg - 64;
    else
      j = 360 + (compass_hdg - 64);

    y = 20;
    for (int i=0; i < 128; i++)
    {
      switch (j)
      {
        case 0:
          u8x8.setFont(u8g2_font_ncenB18_tr);
          u8x8.drawStr(i - 8,y + 10,"N");
          break;
        case 30:
          u8x8.setFont(u8g2_font_ncenB12_tr);
          u8x8.drawStr(i - 5,y +5,"3");
          break;
        case 60:
          u8x8.setFont(u8g2_font_ncenB12_tr);
          u8x8.drawStr(i - 5,y +5,"6");
          break;     
        case 90:
          u8x8.setFont(u8g2_font_ncenB18_tr);
          u8x8.drawStr(i  - 8,y + 10,"E");
          break;
        case 120:
          u8x8.setFont(u8g2_font_ncenB12_tr);
          u8x8.drawStr(i - 11,y +5,"12");
          break;
        case 150:
          u8x8.setFont(u8g2_font_ncenB12_tr);
          u8x8.drawStr(i - 11,y +5,"15");
          break;
        case 180:
          u8x8.setFont(u8g2_font_ncenB18_tr);
          u8x8.drawStr(i - 8,y + 10,"S");
          break;
        case 210:
          u8x8.setFont(u8g2_font_ncenB12_tr);
          u8x8.drawStr(i - 11,y +5,"21");
          break;
        case 240:
          u8x8.setFont(u8g2_font_ncenB12_tr);
          u8x8.drawStr(i - 11,y +5,"24");
          break;        
        case 270:
          u8x8.setFont(u8g2_font_ncenB18_tr);
          u8x8.drawStr(i - 8,y + 10,"W");
          break;
        case 300:
          u8x8.setFont(u8g2_font_ncenB12_tr);
          u8x8.drawStr(i - 11,y +5,"30");
          break;
        case 330:
          u8x8.setFont(u8g2_font_ncenB12_tr);
          u8x8.drawStr(i - 11,y +5,"33");
          break;
        default:
          break;
      }
      //Draw heading bug
       if (j == ap_hdg_bug)
       {
        u8x8.drawBox(i-4,29,8,2);
        u8x8.drawLine (i-2, 28, i+2, 28);
       }
         //
          
      //Draw ticks
      if (j % 30 == 0)
        u8x8.drawLine (i, 0, i, 6);
      else if (j % 5 == 0)
        u8x8.drawLine (i, 0, i, 3);
      if (j++ > 359)
        j = 0;
    }

    //Draw static indicator
    u8x8.setDrawColor(2);
    u8x8.drawTriangle(56,0,72,0,64,10);
    u8x8.setDrawColor(1);
    
    u8x8.sendBuffer();
}

void lcd_update()
{
  //Only update the LCD occasionally
  if (millis() < (lcd_millis + LCD_DELAY))
    return;
  lcd_millis = millis();

//  u8x8_draw_fuel();
  u8x8_draw_compass ();
  /*
    if (battery_bus_sw == 0 || avionics_sw == 0)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Turn on battery");
      lcd.setCursor(0,1);
      lcd.print("and avionics");
      
    }
    else  
    {
    */
    
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(altitude);
      lcd.setCursor(10,0);
      lcd.print(ap_target_altitude);
      
      lcd.setCursor(0,1);     
      lcd.print(adf1);
      lcd.setCursor(19-4,1);
      lcd.print(adf1_standby);
      
      lcd.setCursor(0,2);
      lcd.print(indicated_as);
      lcd.setCursor(0,3);
      lcd.print(ap_altitude_mode);
      lcd.setCursor(10,3);
      lcd.print(ap_altitude_mode);
      
    //}    
    
}

void led_update()
{
  if (ap_passive_mode)
    digitalWrite(LED_AP_PASSIVE_MODE, HIGH);
  else
    digitalWrite(LED_AP_PASSIVE_MODE, LOW);
  
  if (ap_altitude_mode > AP_ALT_PITCH)
    digitalWrite(LED_AP_ALT_MODE, HIGH);
  else
    digitalWrite(LED_AP_ALT_MODE, LOW);

  if (ap_heading_mode > AP_HDG_WINGLEVELER)
    digitalWrite (LED_AP_HDG_MODE, HIGH);
  else
    digitalWrite (LED_AP_HDG_MODE, LOW);
}
