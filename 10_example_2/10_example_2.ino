#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;

unsigned long time1, time2, time3;
//int degree[4] = {0, 90, 180, 90};
int degree[2] = {0, 180};
int num = 0;

void setup() {
  //unsigned long time1, time2;
  myservo.attach(PIN_SERVO);
  Serial.begin(57600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  num = (num+1)%2;
  //time1=millis();
  myservo.write(degree[num]);
  
  /*
  time1=micros();
  time2=micros();
  time3=time2 - time1;
  
  Serial.print("time1:");
  Serial.println(time1);


  
  Serial.print(" time2:");
  Serial.print(time2);
  Serial.print(" total time:");
  Serial.print(time3);
  Serial.print(" degree per second:");
  Serial.println(double(180/time3)*1000000);
  */
  
  delay(385);
}
