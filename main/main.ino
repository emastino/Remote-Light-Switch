// Use a remote controller + servo to turn a light switch on an off


// Source Code + Diagrams for IR Remote
//https://create.arduino.cc/projecthub/electropeak/use-an-ir-remote-transmitter-and-receiver-with-arduino-1e6bc8



// Source for Servo Code
/*      
*  IR read codes     
*  by Hanie kiani     
*  https://electropeak.com/learn/        
*/     
#include <IRremote.h>
#include <Servo.h>

// Light switch button
const byte interruptPin = 2;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 200;    // the debounce time; increase if the output flickers


//Light switch servo
const byte IR_RECEIVE_PIN = 7;
const byte servoPin = 9;
int controllerInput;
int prevState;

// LED assignments
#define greenLED 10
#define redLED 11

// create servo object
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

void setup()
{
    Serial.begin(115200);
    Serial.println("IR Receive test");
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    myservo.attach(servoPin); // use pin 9 for servo commands (need a PWM pin)
    myservo.write(0);
    pinMode(greenLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    
    digitalWrite(redLED, HIGH);

    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), lightFlip, FALLING);
}

void loop()
{
   if (IrReceiver.decode()){
    
      // controller input
      controllerInput = IrReceiver.decodedIRData.command;

      Serial.println(controllerInput);

      // if 22, this means that "0" was pressed
      if(controllerInput == 22){
        if (controllerInput != prevState){
          Serial.println("TURNING LIGHTS OFF");
          myservo.write(0); 
          digitalWrite(greenLED, LOW);
          digitalWrite(redLED,HIGH);
          prevState = controllerInput;
          delay(200);
        }
        
      }
      // if 12, this means that "0" was pressed
      if (controllerInput ==  12){
        if(prevState != controllerInput){
          Serial.println("TURNING LIGHTS ON");
          myservo.write(50);
          digitalWrite(greenLED, HIGH);
          digitalWrite(redLED,LOW);
          prevState = controllerInput;
          delay(200);
        }
        
      }

      IrReceiver.resume();
   }
 
}



void lightFlip() {
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
  // whatever the reading is at, it's been there for longer than the debounce
  // delay, so take it as the actual current state:

      // if 22, this means that "0" was pressed
      if (prevState == 12){
        Serial.println("TURNING LIGHTS OFF");
        myservo.write(0); 
        prevState = 22;

        digitalWrite(greenLED, LOW);
        digitalWrite(redLED,HIGH);
      }
        
      else{
        Serial.println("TURNING LIGHTS ON");
        myservo.write(50);
        prevState = 12;

        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED,LOW);
      }
        
   lastDebounceTime = millis();

  }
}
