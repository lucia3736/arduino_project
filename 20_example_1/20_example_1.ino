#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0

#define _DUTY_NEU 1350
#define _DUTY_LOW 1470
#define _DUTY_HIGH 1250

#define _SERVO_SPEED 30
#define INTERVAL 25

//int speed[4] = {_DUTY_NEU, _DUTY_LOW, _DUTY_HIGH, _DUTY_LOW};
Servo myservo;
int num=0;
float val_ema, alpha, val_ema_1=0;
unsigned long last_sampling_time;
//int duty_chg_per_interval;
int a,b;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(PIN_SERVO);
  
  //duty_chg_per_interval = (_DUTY_HIGH - _DUTY_LOW) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);
  Serial.begin(57600);

  a = 70;
  b = 300;
  last_sampling_time = 0;
}

float ir_distance(void)
{
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0)*10.0;
  /*
  if(volt==0)
  {
    val_ema = val_ema_1;
  }
  else
  {
    val_ema = (alpha*val)+((1-alpha)*val_ema_1);
    val_ema_1 = val_ema;
  }
  */
  return val;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() < last_sampling_time + INTERVAL ) return ;
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali); 
  /*
  if(raw_dist < 240)
    myservo.writeMicroseconds(_DUTY_LOW);
   else if (raw_dist > 240 && raw_dist < 260)
    myservo.writeMicroseconds(_DUTY_NEU);
  else
    myservo.writeMicroseconds(_DUTY_HIGH);
   */

   if (dist_cali < 255)
    myservo.writeMicroseconds(_DUTY_LOW);
   else
    myservo.writeMicroseconds(_DUTY_HIGH);

  last_sampling_time += INTERVAL;
  
}
