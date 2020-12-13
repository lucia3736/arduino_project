//비례제어

#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255 
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.5

#define _DUTY_NEU 1410
#define _DUTY_LOW 1500
#define _DUTY_HIGH 1250


#define INTERVAL 25

#define _INTERVAL_DIST 15
#define _INTERVAL_SERVO 15
#define _INTERVAL_SERIAL 100

#define _KD 300
#define _KP 5
#define _SERVO_ANGLE 25.0
#define _SERVO_SPEED 60.0
//int speed[4] = {_DUTY_NEU, _DUTY_LOW, _DUTY_HIGH, _DUTY_LOW};
Servo myservo;
int num=0;
float val_ema, alpha, val_ema_1=0;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
unsigned long last_sampling_time;
int duty_chg_per_interval;
int a,b;
float dist_target, dist_raw, dist_cali, dist_ema = 0;
int duty_target, duty_curr, duty_curr1;
float error_curr, error_prev, control, pterm, dterm, iterm;
bool event_dist, event_servo, event_serial;
int change, number =0;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(PIN_SERVO);
  
  //duty_chg_per_interval = (_DUTY_HIGH - _DUTY_LOW) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);
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
  a = 10; //-20
  b = 255; //150
  last_sampling_time = 0;
  //duty_chg_per_interval = (_DUTY_HIGH - _DUTY_LOW)*(_SERVO_SPEED / (_SERVO_ANGLE*2))*(_INTERVAL_SERVO/1000.0);
}


float ir_distance(void)
{
  float value;
  float volt = float(analogRead(PIN_IR));
  
  value = ((6762.0/volt-9.0)-4.0)*10.0;
  //return 300.0 / (b-a)*(value - a) + 100;
  value = abs(value);
  return 100 + 300.0 / (b - a) * (value - a); //보정된 값  

}

float ir_distance_filtered(void)
{
  dist_raw = ir_distance();
  if(dist_ema ==0) dist_ema = dist_raw;
  
  return _DIST_ALPHA*dist_raw+(1-_DIST_ALPHA)*dist_ema;
}

void loop() {
  // put your main code here, to run repeatedly:
  //if(millis() < last_sampling_time + INTERVAL ) return ;
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST)
  {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO)
  {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL)
  {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  dist_ema = ir_distance_filtered(); 
  dist_raw = dist_ema;
  
  if(event_dist)
  {
    event_dist = false;
    //dist_ema = ir_distance_filtered();
    //error_curr = dist_raw - _DIST_TARGET;
    error_curr = _DIST_TARGET - dist_raw;
    
    //pterm =  duty_curr - duty_target;
    pterm = error_curr;
    
    //dterm = _KD * (error_curr - error_prev);
    
    dterm = 0;
    iterm = 0;
    
    //control = dterm + pterm;
    
    control = _KP*pterm + iterm + dterm*_KD;
    //duty_target = _DUTY_NEU + control;
    duty_target = _DUTY_NEU + control * ((control>0)?(_DUTY_HIGH - _DUTY_NEU):(_DUTY_NEU - _DUTY_LOW));
    // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
    
    if(duty_target < _DUTY_LOW) duty_target = _DUTY_LOW; // lower limit
    if(duty_target > _DUTY_HIGH) duty_target = _DUTY_HIGH; // upper limit
    // update error_prev
    error_prev = error_curr;
  }
  if(event_servo)
  {
    event_servo = false;


    if(dist_ema <= 240)
    {
      duty_curr = duty_target + pterm*1.5;
    }
    else if (dist_ema <= 240)
    {
      duty_curr = duty_target + pterm;
      if(duty_curr > _DUTY_NEU) duty_curr = duty_target;
    }

    else if(dist_ema <= 265)
    {
      duty_curr = duty_target;
    }
    else if(dist_ema <= 300)
    {
      duty_curr = duty_target - pterm;
    }
    else
      duty_curr = duty_target - pterm*1.2;
    
    
    //duty_curr1 = duty_curr;
    myservo.writeMicroseconds(duty_curr);
  }
  if(event_serial)
  {
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
  
}
