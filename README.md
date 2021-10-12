# IOT

IOT LAB
1.Blink
Connect the cable to cpu and Arduino board and led(high end in 13 and low end in ground)
Arduinouno-examples-blink-upload-output

2.tempaerture sensor
Uno board-analog side-type 2 pin-vcc(1) to 5v,data(2) ie.output to A0,ground to ground
Add zip file -tools-include lib-zip-in pgm a line is incl,so delete 2nd line
Program run-window opens-op

3.light sensor
Uno board-analog side-type 2 pin-vcc(1) to 5v,data(2)(A0) ie.output to A0,ground to ground,digital side-led
Program run

4.ultrasonic sensor
Speaker-vcc to 5v,trig to 10,echo to 9,gnd exception(any one only needed)
Sensor-gnd to gnd,I/O(buzz pin) to 2,vcc to 5v
Op-low sound when sensor covered and vice versa,sound only when there is obstacle for 50m and vice versa

Light Sensor:

const int ledPin = 13;
const int ldrPin = A0;
void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(ldrPin, INPUT);
}

void loop() {
  int ldrStatus = analogRead(ldrPin);

  if (ldrStatus <= 400)
  {
    digitalWrite(ledPin, HIGH);
    Serial.print("Its Dark, Turn on the LED:");
    Serial.println(ldrStatus);

  }
  else
  {
    digitalWrite(ledPin, LOW);
    Serial.print("Its Bright, Turn off the LED:");
    Serial.println(ldrStatus);
  }

}

----------------------------------------------------------------------------------------------------------

Temperature Sensor:

#include <dht.h>


#include "dht.h"
#define dht_apin A0 // Analog Pin sensor is connected to
 
dht DHT;
 
void setup(){
 
  Serial.begin(9600);
  delay(500);//Delay to let system boot
  Serial.println("DHT11 Humidity & temperature Sensor\n\n");
  delay(1000);//Wait before accessing Sensor
 
}//end "setup()"
 
void loop(){
  //Start of Program
 
    DHT.read11(dht_apin);
   
    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature);
    Serial.println("C  ");
   
    delay(5000);//Wait 5 seconds before accessing sensor again.
 
  //Fastest should be once every two seconds.
 
}// end loop()

----------------------------------------------------------------------------------------------------------

LED:

int led =13;
void setup() {
pinMode(led,OUTPUT);
}
void loop() {
  digitalWrite(led,HIGH);
  delay(1000);
  digitalWrite(led,LOW);
  delay(1000);

}
------------------------------------------------------

TOUCH SENSOR:
#define ctsPin 2

// Pin for capactitive touch sensor

int ledPin = 13;

// pin for the LED

void setup()

{

Serial.begin(9600);

pinMode(ledPin, OUTPUT);

pinMode(ctsPin, INPUT);

}

void loop()

{

int ctsValue = digitalRead(ctsPin);

if (ctsValue == HIGH)

{

digitalWrite(ledPin, HIGH);

Serial.println("TOUCHED");

}

else{

digitalWrite(ledPin,LOW);

Serial.println("not touched");

}

delay(500);

}

------------------------------------------------------


// ---------------------------------------------------------------- //
// Arduino Ultrasoninc Sensor HC-SR04
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using HC-SR04 Module
// Tested on 17 September 2019
// ---------------------------------------------------------------- //

#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
}
void loop() {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}


------------------------------------------------------

IR SENSOR

int IRSensor = 2; // connect ir sensor to arduino pin 2
int LED = 13; // conect Led to arduino pin 13



void setup()
{



  pinMode (IRSensor, INPUT); // sensor pin INPUT
  pinMode (LED, OUTPUT); // Led pin OUTPUT
}

void loop()
{
  int statusSensor = digitalRead (IRSensor);
 
  if (statusSensor == 1)
  {
    digitalWrite(LED, LOW); // LED LOW
  }
 
  else
  {
    digitalWrite(LED, HIGH); // LED High
  }
 
}


---------------------------------------------------
ULTRASONIC WITH BUZZER

/*
  This code should work to get warning cross the buzzer when something be closer than 0.5 meter
  Circuit is ultrasonic sensor and buzzer +5v and Arduino uno is used
  a_atef45@yahoo.com
  www.zerosnones.net
  +201153300223
*/
// Define pins for ultrasonic and buzzer
int const trigPin = 10;
int const echoPin = 9;
int const buzzPin = 2;

void setup()
{
  pinMode(trigPin, OUTPUT); // trig pin will have pulses output
  pinMode(echoPin, INPUT); // echo pin should be input to get pulse width
  pinMode(buzzPin, OUTPUT); // buzz pin is output to control buzzering
}

void loop()
{
  // Duration will be the input pulse width and distance will be the distance to the obstacle in centimeters
  int duration, distance;
  // Output pulse with 1ms width on trigPin
  digitalWrite(trigPin, HIGH);
  delay(1);
  digitalWrite(trigPin, LOW);
  // Measure the pulse input in echo pin
  duration = pulseIn(echoPin, HIGH);
  // Distance is half the duration devided by 29.1 (from datasheet)
  distance = (duration/2) / 29.1;
  // if distance less than 0.5 meter and more than 0 (0 or less means over range)
    if (distance <= 10 && distance >= 0) {
      // Buzz
      digitalWrite(buzzPin, HIGH);
    } else {
      // Don't buzz
      digitalWrite(buzzPin, LOW);
    }
    // Waiting 60 ms won't hurt any one
    delay(60);
}

