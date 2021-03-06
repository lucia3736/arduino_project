#define PIN_LED 13
unsigned int count, toggle;

//LED turns off/on in 1 second
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
  digitalWrite(PIN_LED, toggle); //turn off LED
}

void loop()
{
  Serial.println(++count);
  toggle=toggle_state(toggle); //toggle LED value
  digitalWrite(PIN_LED, toggle); //update LED status
  delay(1000); //wait for 1000 milliseconds
}

int toggle_state(int toggle)
{
  toggle++;
  return toggle%2;
}
