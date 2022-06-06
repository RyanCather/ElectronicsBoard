// SD Card Module
#include <SPI.h>
#include <SD.h>

// DC Motor & Motor Module - L298N
#include <L298N.h>

// Pin definition
const unsigned int IN1 = 7;
const unsigned int IN2 = 8;
const unsigned int EN = 9;

// Create one motor instance
L298N motor(IN1, IN2);

// Moisture Sensor
#define moisturePin A5

// Line Sensor
#define lineSensorPin 3    // Line Sensor (light). HIGH or LOW values.

// GPS
/* RXPin & TXPin
 * If a Uno, then 4 and 3
 * If a Mega/Leonardo, then 10 and 11.
 * Check the software Serial documentation for change interrupts
 * https://www.arduino.cc/en/Reference/SoftwareSerial
 */
static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps; // The TinyGPS++ object
SoftwareSerial ss(RXPin, TXPin);// The serial connection to the GPS device

// Real Time Clock (RTC)
#include "RTClib.h"
RTC_Millis rtc;     // Software Real Time Clock (RTC)
DateTime rightNow;  // used to store the current time.

// Traffic Lights - LED Outputs
#define ledRed A0
#define ledYellow A1
#define ledGreen A2

// Sonar - HC-SR04
#define echoPin 22 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 23 //attach pin D3 Arduino to pin Trig of HC-SR04

//Potentiometer
#define pot A3

// Servo
#include <Servo.h>
Servo myservo;

// Piezo Buzzer
#define piezoPin 5

// Crash Sensor / Button
#define crashSensor 7

// SD Card
#define SDpin 53

// IR Remote
#include <IRremote.h>
#define IR_INPUT_PIN    2
IRrecv irrecv(IR_INPUT_PIN);
decode_results results;

#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

void setup() {
  Serial.begin(9600);           // Open serial communications and wait for port to open:
  while (!Serial) {
    delay(1);                   // wait for serial port to connect. Needed for native USB port only
  }

  // SD Card initialisation
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDpin)) {
    Serial.println("initialization failed!");
    while (1);
  }

  // Real Time Clock (RTC)
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));

  logEvent("System Initialisation Start");

  // Traffic Lights - LED Outputs

  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  logEvent("Init - LEDs");

  // GPS
  ss.begin(GPSBaud);
  logEvent("Init - GPS");

  //Potentiometer
  pinMode(pot, INPUT);
  logEvent("Init - Potentiometer");

  // Piezo Buzzer
  pinMode(piezoPin, OUTPUT);
  logEvent("Init - Piezo");

  // DC Motor & Motor Module - L298N
  motor.setSpeed(70);
  logEvent("Init - DC Motor");

  // Moisture Sensor
  pinMode(moisturePin, INPUT);
  logEvent("Init - Moisture Pin");

  // Servo
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  logEvent("Init - Servo");

  // Line Sensor
  pinMode(lineSensorPin, OUTPUT);
  logEvent("Init - Line Sensor");

  // Sonar - HC-SR04
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  logEvent("Init - Sonar");

  //Built in LED
  pinMode(13, OUTPUT);
  logEvent("Init - Built-in LED");

  // Crash Sensor / Button
  pinMode(crashSensor, INPUT);
  logEvent("Init - Crash Sensor");
  SPI.begin();

  // IR
  irrecv.enableIRIn();

  logEvent("Init - IR Sensor");

  logEvent("System Initialisation Complete...");
}

void loop() {
//  doorAlarm();            // sonar and servo
  smartHeatingSystem();     // IR remote & DC motor
  coffeeMachine();          // GPS and & LEDs
  remoteDecode();           // IR 
  locationBarrier();        // GPS
  delay(100);
}

/*
   Sets the alarm (buzzer) off if someone/thing is too close (sonar).

   @param null
   @return null or void
*/
void doorAlarm() {
  int doorThreshold = 1;
  int doorDistance = getSonarDistance();
  if (doorDistance < doorThreshold) {

    // disabled noise output due to annoyance.
    tone(piezoPin, 1000); // Send 1KHz sound signal...
    delay(100);
    tone(piezoPin, 500); // Send 1KHz sound signal...
    delay(100);
    String distanceStr = String(doorDistance);
    String thresholdStr = String(doorThreshold);
    String eventToLog = "Alarm activated, " + distanceStr + "<" + thresholdStr;
    logEvent(eventToLog);
  } else {
    noTone(piezoPin);
  }
}

/**
  Indicates whether the object is too close to the sensor.

  @param distance value read from the distance sensor
  @return true if object is too close, false otherwise.
*/
boolean isObjectTooClose(int distance) {
  //  if (distance > threshold) {
  //    return true;
  //  } else {
  //    return false;
  //  }

}

void smartHeatingSystem() {

}

void coffeeMachine() {

}


/*
   Gets the value given by the Keyes IR remote.
   Code values are:

   Up     : 25245
   Down   : -22441
   Left   : 8925
   Right  : -15811
   Ok     : 765
   1      : 26775
   2      : -26521
   3      : -20401
   4      : 12495
   5      : 6375
   6      : 31365
   7      : 4335
   8      : 14535
   9      : 23205
   0      : 19125
   #      : 21165
          : 17085

   Test against each code and perform required action. See example in code.

   @params: None
   @return: void
*/
void remoteDecode() {

  if (irrecv.decode(&results)) {

    int code = results.value;
    //    Serial.println(code);
    if (code == 25245) {  // Up
      Serial.println("Up");
    }
    irrecv.resume();
  }
}

int getSonarDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  return distance;
}


void motorDC() {
  motor.forward();
  delay(1000);
  motor.stop();
  delay(1000);
  motor.backward();
  delay(1000);

}


/*
   Gets the GPS coords and tests whether it's in "bounds"

   @params: none
   @return: void
*/
void locationBarrier() {
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      getGPSInfo();
}

void getGPSInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.println(F("INVALID"));
  }
}

void logEvent(String dataToLog) {
  /*
     Log entries to a file on an SD card.
  */
  // Get the updated/current time
  DateTime rightNow = rtc.now();

  // Open the log file
  File logFile = SD.open("events.csv", FILE_WRITE);
  if (!logFile) {
    Serial.print("Couldn't create log file");
    abort();
  }

  // Log the event with the date, time and data
  logFile.print(rightNow.year(), DEC);
  logFile.print(",");
  logFile.print(rightNow.month(), DEC);
  logFile.print(",");
  logFile.print(rightNow.day(), DEC);
  logFile.print(",");
  logFile.print(rightNow.hour(), DEC);
  logFile.print(",");
  logFile.print(rightNow.minute(), DEC);
  logFile.print(",");
  logFile.print(rightNow.second(), DEC);
  logFile.print(",");
  logFile.print(dataToLog);

  // End the line with a return character.
  logFile.println();
  logFile.close();
  Serial.print("Event Logged: ");
  Serial.print(rightNow.year(), DEC);
  Serial.print(",");
  Serial.print(rightNow.month(), DEC);
  Serial.print(",");
  Serial.print(rightNow.day(), DEC);
  Serial.print(",");
  Serial.print(rightNow.hour(), DEC);
  Serial.print(",");
  Serial.print(rightNow.minute(), DEC);
  Serial.print(",");
  Serial.print(rightNow.second(), DEC);
  Serial.print(",");
  Serial.println(dataToLog);
}
