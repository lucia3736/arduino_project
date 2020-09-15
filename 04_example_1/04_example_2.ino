void setup()
{
  Serial.begin(115200);
  while(!Serial)
  {
    ; //wait for serial port to connect. 
  }
}

void loop()
{
  Serial.println("Hello World!");
  delay(1000);
}
