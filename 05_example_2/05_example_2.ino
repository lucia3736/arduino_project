#define PIN_LED 13
unsigned int count, toggle;

//LED turns on for 1 second
//Then LED turns off/on for 5 times for 1 second
void setup()
{
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); //initialize serial port
  while(!Serial)
  {
    ; //wait for serial port to connect
  }
  Serial.println("Hello World!");
  count=toggle=0;
  digitalWrite(PIN_LED, 1);
  delay(1000);//turn on LED for 1 second
}

void loop()
{
  Serial.println(++count);
  if(count==12){while(1){digitalWrite(PIN_LED,0);}}
  //After LED turns on/off for 5 times, turn off LED
  toggle=toggle_state(toggle);//toggle LED value
  digitalWrite(PIN_LED, toggle); //update LED status
  delay(100); //wait for 100 milliseconds
}

int toggle_state(int toggle)
{
  toggle++;
  return toggle%2;
}
