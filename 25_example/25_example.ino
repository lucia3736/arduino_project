//비례미분제어

#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 230 
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

#define _KP 1.5
#define _KD 34
#define _KI 1.3

#define _SEQ_SIZE 8

#define _SERVO_ANGLE 30.0
#define _SERVO_SPEED 300.0
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

float x[_SEQ_SIZE] = {0.0, 10.0, 70.0, 170.0, 300.0, 280.0, 400.0, 470.0};

// real values
float y[_SEQ_SIZE] = {100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0, 450.0};

void setup() {
  // put your setup code here, to run once:
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DIST_TARGET);
  //duty_chg_per_interval = (_DUTY_HIGH - _DUTY_LOW) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);
  duty_chg_per_interval = (_DUTY_LOW- _DUTY_HIGH) * (_SERVO_SPEED / (_SERVO_ANGLE * 2)) * (_INTERVAL_SERVO / 1000.0);
  Serial.begin(57600);
  last_sampling_time_dist=0;
  last_sampling_time_servo=0;
  last_sampling_time_serial=0;
  duty_target = _DUTY_NEU;
  duty_curr = _DUTY_NEU;
  error_prev = 0;
  pterm = iterm = dterm = 0;
  //a = 70;
  //b = 300;
  a = -20;
  b = 400;
  last_sampling_time = 0;
  //duty_chg_per_interval = (_DUTY_HIGH - _DUTY_LOW)*(_SERVO_SPEED / (_SERVO_ANGLE*2))*(_INTERVAL_SERVO/1000.0);
}


float ir_distance(void)
{
  float value;
  float volt = float(analogRead(PIN_IR));

  value = ((6762.0/volt-9.0)-4.0)*10.0;
  //return 300.0 / (b-a)*(value - a) + 100;
  return 100 + 300.0 / (b - a) * (value - a);

}

float ir_distance_sequence(void)
{
  float value, real_value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/volt-9.0)-4.0)*10.0;

  int s = 0, e = _SEQ_SIZE -1, m;

  while(s<=e)
  {
    m = (s+e)/2;
    if(value < x[m])
    {
      e = m-1;
    }
    else if (value > x[m+1])
    {
      s = m+1;
    }
    else break;
  }
  if(s>e)
  {
    if(value > 1000.0 || value < 12000.0) real_value = _DIST_TARGET;
    else if (s==0) real_value = _DIST_MIN;
    else real_value = _DIST_MAX;
  }
  real_value = (y[m+1] - y[m])/(x[m+1]-x[m])*(value-x[m])+y[m];
  return real_value;
}

float ir_distance_filtered(void)
{
  dist_raw = ir_distance_sequence();
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

  dist_raw = ir_distance_filtered(); 
  dist_ema = dist_raw;

  if(event_dist)
  {
    event_dist = false;
    //dist_ema = ir_distance_filtered();
    error_curr = dist_raw - _DIST_TARGET;

    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    
    control = dterm + pterm + iterm;
    //duty_target = _DUTY_NEU + control;
    
    duty_target = _DUTY_NEU + control * ((control >0)?(_DUTY_LOW - _DUTY_NEU):(_DUTY_NEU-_DUTY_HIGH));
    // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_HIGH) duty_target = _DUTY_HIGH; // lower limit
    if(duty_target > _DUTY_LOW) duty_target = _DUTY_LOW; // upper limit
    // update error_prev
    error_prev = error_curr;
  }
  if(event_servo)
  {
    event_servo = false;

    
    if(duty_target>duty_curr) 
    {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else 
    {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    
    /*
    if(dist_ema <= 210)
      duty_curr = duty_target + (255 - dist_ema)*2;
    else
      duty_curr = duty_target - (dist_ema - 255)*3;
    */
    
    //duty_curr1 = duty_curr;
    myservo.writeMicroseconds(duty_curr);
  }
  if(event_serial)
  {
    event_serial = false;
    

    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    
  }
  
}
