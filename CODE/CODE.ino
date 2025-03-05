 
#include <Wire.h>
#define ENABLE_DATALOGGER 0

#define LOAD_ALGORITHM 0
////////////////////////Chân đo tín hiệu////////////
#define SOL_AMPS_CHAN A0
#define SOL_VOLTS_CHAN A1
#define BAT_VOLTS_CHAN A3
#define BAT_AMPS_CHAN A2

#define AVG_NUM 8
#define SOL_AMPS_SCALE  0.026393581
#define SOL_VOLTS_SCALE 0.7
#define BAT_VOLTS_SCALE 0.0
#define PWM_PIN 10                ///Chân PWM là chân 10

#define PWM_FULL 250
#define PWM_MAX 200
#define PWM_MIN 60
#define PWM_START 100
#define PWM_INC 1
#define OFF_NUM 4
#define TRUE 1
#define FALSE 0
#define ON TRUE
#define OFF FALSE

#define TURN_ON_MOSFETS digitalWrite(PWM_ENABLE_PIN, HIGH)
#define  digitalWrite(PWM_ENABLE_PIN, LOW)

#define ONE_SECOND 5000
/*
#define LOW_SOL_WATTS 30.00
#define MIN_SOL_WATTS 0.0005
#define MIN_BAT_VOLTS 11.5
#define MAX_BAT_VOLTS 13
#define BATT_FLOAT 12
#define LVD 11.5
*/
#define MIN_SOL_VOL 10 
#define LOW_SOL_WATTS 0.0005
#define MIN_SOL_WATTS 0.0005
#define MIN_BAT_VOLTS 11
#define MAX_BAT_VOLTS 12.6
#define BATT_FLOAT 12
#define LVD 11.5
#define BACK_LIGHT_PIN D13
float sol_amps;
float sol_volts;
float bat_volts;
float sol_watts;
float bat_amps;
float bat_watts;
float old_sol_watts = 0;
unsigned int seconds = 0;
unsigned int prev_seconds = 0;
unsigned int interrupt_counter = 0;
unsigned long time = 0,time1 =0;
int delta = PWM_INC;
int pwm = 0;
int back_light_pin_State = 0;
boolean load_status = false;

enum charger_mode {off, on, bulk, bat_float} charger_state;

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

#include <SoftwareSerial.h>
 

SoftwareSerial mySerial (11, 12);
void setup()
{ 
  Wire.begin();
  
  pinMode(PWM_PIN, OUTPUT);
  charger_state = on;
 mySerial.begin(9600);
  Serial.begin(9600);
  pwm = PWM_START;
  delay(100);  // This delay is needed to let the display to initialize
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();
}


void loop()
{ if (mySerial.available() > 0) {
      char data =  mySerial.read();
     if(data == 'U'){
      Serial.println(sol_volts);
     }
     if(data == 'I'){
      Serial.println(sol_amps);
     }
    }
  read_data();
  run_charger();
  set_pwm_duty();
  print_data();
  load_control();
  if(millis() - time1 > 1000){
    display.clearDisplay();
  hienthi();
  time1 = millis();
  }
  
}


 


void read_data(void)
{ float sum1,sum2,sum3,sum4,old1,old2,old3,old4;
  for(int s =0; s <= 50;s++){
  float val1 = analogRead(SOL_AMPS_CHAN);
  float val2 = analogRead(SOL_VOLTS_CHAN);
  float val3 = analogRead(BAT_VOLTS_CHAN);
  float val4 = analogRead(BAT_AMPS_CHAN);
  sum1 += val1;
  sum2 += val2;
  sum3 += val3;
  sum4 += val4;
  delay(1);
  }
  old1 = sum1/50;
  old2 = sum2/50;
  old3 = sum3/50;
  old4 = sum4/50;
  sol_amps = old1/1023 * 40;
  sol_amps = sol_amps - 20.4;
  if(sol_amps <=0 ) sol_amps =0;
  bat_amps = old4/1023 * 40;
  bat_amps = bat_amps - 20.35;
  if(bat_amps <=0 ) bat_amps =0;
  sol_volts = (old2/1023 * 36.3)-SOL_VOLTS_SCALE;
  bat_volts = (old3/1023 * 36.3)-BAT_VOLTS_SCALE;
  if(bat_volts <=0 ) bat_volts  =0;
  if(sol_volts <=0 ) sol_volts =0;
  sol_watts = sol_amps * sol_volts ;
  bat_watts = bat_amps * bat_volts ;
  
}



void set_pwm_duty(void)
{

  if (pwm > PWM_MAX) {
    pwm = PWM_MAX;
  }
  else if (pwm < 15) {
    pwm = 0;
  }
  analogWrite(PWM_PIN, pwm);
  
}


void run_charger(void)
{

  static int off_count = OFF_NUM;

  switch (charger_state)
  {
    case on:
      if (sol_volts < 15)
      {
        charger_state = off;
         
        pwm = 0;
       
        off_count = OFF_NUM;
        ;
      }
      else if(bat_volts >= MAX_BAT_VOLTS){
        pwm = 0;
        charger_state = off;
      }
      else if (bat_volts > (BATT_FLOAT - 0.1))
      {
        charger_state = bat_float;
        
      }
      else if (sol_watts < LOW_SOL_WATTS) {
        pwm = PWM_MAX;
        
        set_pwm_duty();
      }
      else {
        pwm = bat_volts * 16.5;
        charger_state = bulk;
        
      }
      break;
    case bulk:
      if (sol_volts < 15)
      {
        charger_state = off;
        
        pwm = 0;
        off_count = OFF_NUM;
        
      }
      else if (bat_volts > BATT_FLOAT)
      {
        charger_state = bat_float;
        
      }
      else if (sol_watts < LOW_SOL_WATTS)
      {
        charger_state = on;

      }
      else {
        if (old_sol_watts >= sol_watts)
        {
          delta = -delta;
        }
        
        pwm += delta;
        old_sol_watts = sol_watts;
        set_pwm_duty();
      }
      break;
    case bat_float:

      if (sol_volts < 10)
      {
        charger_state = off ;
        off_count = OFF_NUM;
        
        pwm = 0;
        ;
        
        set_pwm_duty();
      }
      else if(bat_volts >= MAX_BAT_VOLTS){
        pwm = 0;
        charger_state = off;
      }
      else if (bat_volts > BATT_FLOAT)
      {
        pwm = PWM_MIN;
        set_pwm_duty();
      }
      else if (bat_volts < BATT_FLOAT)
      {
        pwm = PWM_MAX;
        
        set_pwm_duty();

        if (bat_volts < (BATT_FLOAT - 0.1))
        {
          charger_state = bulk;
        }
      }
      break;
    case off:
      ;
      if (off_count > 0)
      {
        off_count--;
      }
      else if(bat_volts >= MAX_BAT_VOLTS){
        pwm = 0;
        charger_state = off;
      }
      else if ((bat_volts > BATT_FLOAT) && (sol_volts > bat_volts)) {
        charger_state = bat_float;

      }
      else if ((bat_volts > MIN_BAT_VOLTS) && (bat_volts < BATT_FLOAT) && (sol_volts > bat_volts)) {
        charger_state = bulk;

      }
      break;
    default:
      ;
      break;
  }
}



void load_control()
{
#if LOAD_ALGORITHM == 0
  load_on(sol_watts < MIN_SOL_WATTS && bat_volts > LVD);
#else
  load_on(sol_watts > MIN_SOL_WATTS && bat_volts > BATT_FLOAT);
#endif
}

void load_on(boolean new_status)
{
  if (load_status != new_status)
  {
    load_status = new_status;
    digitalWrite(LOAD_PIN, new_status ? HIGH : LOW);
  }
}

void print_data(void)
{
/*
  Serial.print(seconds, DEC);
  Serial.print("      ");

  Serial.print("Charging = ");
  if (charger_state == on) Serial.print("on   ");
  else if (charger_state == off){ Serial.print("off  ");
        pwm = 0;}
  else if (charger_state == bulk) Serial.print("bulk ");
  else if (charger_state == bat_float) Serial.print("float");
  Serial.print("      ");

  Serial.print("pwm = ");
  if (charger_state == off)
    Serial.print(0, DEC);
  else
    Serial.print(pwm, DEC);
  Serial.print("      ");

  Serial.print("Current (panel) = ");
  Serial.print(sol_amps);
  Serial.print("      ");

  Serial.print("Voltage (panel) = ");
  Serial.print(sol_volts);
  Serial.print("      ");

  Serial.print("Power (panel) = ");
  Serial.print(sol_watts );
  Serial.print("      ");

  Serial.print("Battery Voltage = ");
  Serial.print(bat_volts);
  Serial.print("      ");

  Serial.print("\n\r");
*/
}

void light_led(char pin)
{
  static char last_lit;
  if (last_lit == pin)
    return;
  if (last_lit != 0)
    digitalWrite(last_lit, LOW);
  digitalWrite(pin, HIGH);
  last_lit = pin;
}

void hienthi(){
  String tt;
  if(charger_state == off) tt = "OFF";
  if(charger_state == on) tt = "ON";
  if(charger_state == bulk) tt = "BULK";
  if(charger_state == bat_float) tt = "FLOAT";
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("SOL: " + String(sol_volts,1) + "V " + String(sol_amps,1) + "A " + String(sol_watts,1)+"W");
   display.setCursor(0,10);
  display.println("BAT: " + String(bat_volts,1) + "V " + String(bat_amps,1) + "A " + String(bat_watts,1)+"W");
  display.setCursor(0,20);
  display.println("TRANG THAI SAC: " + String(tt));
  display.display();
}
void callback()
{
  if (interrupt_counter++ > ONE_SECOND)
  {
    interrupt_counter = 0;
    seconds++;
  }
}


 
 
