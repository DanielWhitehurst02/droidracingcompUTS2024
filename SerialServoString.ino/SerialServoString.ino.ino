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

Servo steering;

Servo throttle;

String inputString = "";         // a String to hold incoming data
String inputString2 = "";
bool stringComplete = false;  // whether the string is complete
bool stringComplete2 = false;

int steer = 90;
int throt = 0;


void setup() {
  // initialize serial:
  Serial.begin(19200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(30);
  steering.attach(9);
  throttle.attach(10);
  
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete && stringComplete2) {
    //Serial.println(inputString2);

    steer = atoi(inputString.c_str());
    throt = atoi(inputString2.c_str());
    //Serial.println(steer);
    // clear the string:
    inputString = "";
    inputString2 = "";
    stringComplete = false;
    stringComplete2 = false;
    
  }

  steering.write(steer);
  throttle.write(throt);
  delay(30);

}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    if(stringComplete){
      inputString2 +=inChar;

     if(inChar == '\n'){
      stringComplete2 = true;
     }
    }
    else{
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
      if (inChar == ',') {
        stringComplete = true;
      }
    }
    
  }
}
