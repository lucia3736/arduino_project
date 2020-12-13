//비례제어

#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 250 
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.1

#define _DUTY_NEU 1420
#define _DUTY_LOW 1500
#define _DUTY_HIGH 1250


#define INTERVAL 25

#define _INTERVAL_DIST 15
#define _INTERVAL_SERVO 15
#define _INTERVAL_SERIAL 100

#define _KD 300

#define _SERVO_ANGLE 30.0
#define _SERVO_SPEED 70.0
//int speed[4] = {_DUTY_NEU, _DUTY_LOW, _DUTY_HIGH, _DUTY_LOW};
Servo myservo;
int num=0;
float val_ema, alpha, val_ema_1=0;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
unsigned long last_sampling_time;
int duty_chg_per_interval;
int a,b;
float dist_target, dist_raw, dist_cali, dist_ema;
int duty_target, duty_curr, duty_curr1;
float error_curr, error_prev, control, pterm, dterm, iterm;
bool event_dist, event_servo, event_serial;
int change, number =0;
float volt;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(PIN_SERVO);
  
  duty_chg_per_interval = (_DUTY_HIGH - _DUTY_LOW) * (_SERVO_SPEED / (_SERVO_ANGLE * 2)) * (_INTERVAL_SERVO / 1000.0);
  Serial.begin(57600);
  last_sampling_time_dist=0;
  last_sampling_time_servo=0;
  last_sampling_time_serial=0;
  duty_target = _DUTY_NEU;
  duty_curr = _DUTY_NEU;
  error_prev = 0;
  //a = 70;
  //b = 300;
  a = -20;
  b = 150;
  last_sampling_time = 0;
  //duty_chg_per_interval = (_DUTY_HIGH - _DUTY_LOW)*(_SERVO_SPEED / (_SERVO_ANGLE*2))*(_INTERVAL_SERVO/1000.0);
}


float ir_distance(void)
{
  float val;
  volt = float(analogRead(PIN_IR));
  val = ((6762.0/volt-9.0)-4.0)*10.0;
  return val;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() < last_sampling_time + INTERVAL ) return ;
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b-a)*(raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
    Serial.print("min:-100,max:500,volt:");
  Serial.print(volt);
  Serial.print(",dist_cali;");
  Serial.println(dist_cali);

  if(dist_cali < 255)
    myservo.writeMicroseconds(_DUTY_LOW);
  else
    myservo.writeMicroseconds(_DUTY_HIGH);
    
  last_sampling_time += INTERVAL;
  
}
