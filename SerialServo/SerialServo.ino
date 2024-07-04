/*
  Serial Event example

  When new serial data arrives, this sketch adds it to a String.
  When a newline is received, the loop prints the string and clears it.

  A good test for this is to try it with a GPS receiver that sends out
  NMEA 0183 sentences.

  NOTE: The serialEvent() feature is not available on the Leonardo, Micro, or
  other ATmega32U4 based boards.

  created 9 May 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/SerialEvent
*/

#include <Servo.h>

Servo steer;

int pos = 0;
int incoming = 90;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  // initialize serial:
  Serial.begin(19200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  steer.attach(9);
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    Serial.print(" I received: ");
    Serial.print(incoming);
    steer.write(pos);
    stringComplete = false;

  }
  delay(15);
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    incoming = Serial.read();
    pos = incoming;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
      stringComplete = true;
    }
  }
