// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
//#define INTERVAL 100 // sampling interval (unit: ms)
#define INTERVAL 25
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300// maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.1 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.
#define N 30 //number of samples to save
// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float queue_array[N] = {0, };
int median, save_num = 0;
int err;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);

  // initialize USS related variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

  // initialize serial port
  Serial.begin(57600);

  // initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
  int fadeValue = 0;
  float cal;
  // wait until next sampling time.
  // millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL) return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  //dist_ema = (alpha*dist_raw) + ((1-alpha)*dist_ema_1);
  //dist_ema_1=dist_ema;

  // output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("median:");
  Serial.print(map(median, 0, 400, 100, 500));
  Serial.print(",");
  Serial.println("Max:500");

  // turn on the LED if the distance is between dist_min and dist_max
  if (dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    cal = (dist_min + dist_max) / 2;
    fadeValue = (abs(cal - dist_raw) * 256) / 100;
    analogWrite(PIN_LED, fadeValue);
  }
  /*
    else if (dist_raw < (dist_min + dist_max)/2) {
    cal = ((3*dist_min)+dist_max)/4;
    fadeValue = ((dist_raw - cal)*256)/100;
    analogWrite(PIN_LED, fadeValue);
    }
    else {
    cal = (dist_min + (3*dist_max))/4;
    fadeValue= ((cal - dist_raw)*256)/100;
    analogWrite(PIN_LED, fadeValue);
    }
  */
  // do something here
  // delay(50); // Assume that it takes 50ms to do something.

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  //float err;
  //int num = N/2; //to calculate medium
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale;
  median = reading; // unit: mm
  //err = abs(reading - queue_array[save_num%N - 1])/queue_array[save_num%N - 1];
  err = abs(reading - queue_array[save_num % N - 1]);
  if (reading < dist_min || reading > dist_max || err > 250) {
    median = queue_array[int(N / 2)];
  }
  queue_array[save_num % N] = median;
  save_num++;
  return reading;
  // Pulse duration to distance conversion example (target distance = 17.3m)
  // - round trip distance: 34.6m
  // - expected pulse duration: 0.1 sec, or 100,000us
  // - pulseIn(ECHO, HIGH, timeout) * 0.001 * 0.5 * SND_VEL
  //           = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
  //           = 100,000 * 0.001 * 0.5 * 346 * micro * sec * milli * meter
  //                                           ----------------------------
  //                                           micro * sec
  //           = 100 * 173 milli*meter = 17,300 mm = 17.3m
  // pulseIn() returns microseconds.
}
