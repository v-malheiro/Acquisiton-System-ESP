/*
  Victor Hugo Enes Malheiro
*/

/* Code to run the stepper motor */

//--------------------------------- LIBRARIES ---------------------------------//
#include <ESP_FlexyStepper.h>
#include <BluetoothSerial.h>
//--------------------------------- LEDS RGB ---------------------------------//
#define LEDR 25
#define LEDG 33 
#define LEDB 32
#define R_channel 0
#define B_channel 1
#define G_channel 2
#define pwm_Frequency 5000
#define pwm_resolution 8
//--------------------------------- MOTOR PINS ---------------------------------//
#define MOTOR_STEP 5
#define MOTOR_DIRECTION 19
//--------------------------------- Acquisition ---------------------------------//
#define ACQUISITION 4   // define acquisition module
//--------------------------------- Buttons ---------------------------------//
#define Button_White 26  // define pin for start
#define Button_Red 12    // define pin for translation in forward
#define Button_Yellow 14 // define pin for translation in backward
#define Button_Blue 27   // define pin for stop

int contador = 0;
int contador2 = 0;

bool init_acquisition = false;

ESP_FlexyStepper stepper;
BluetoothSerial SerialBT;

//--------------------------------- SETUP AND PIN MODES ---------------------------------//
void setup(){
  Serial.begin(115200);
  pinMode(ACQUISITION,OUTPUT);
  pinMode(Button_White, INPUT);
  pinMode(Button_Red, INPUT);
  pinMode(Button_Yellow, INPUT);
  pinMode(Button_Blue, INPUT);

  ledcAttachPin(LEDR,R_channel);
  ledcAttachPin(LEDB,B_channel);
  ledcAttachPin(LEDG,G_channel);

  ledcSetup(R_channel,pwm_Frequency,pwm_resolution);
  ledcSetup(B_channel,pwm_Frequency,pwm_resolution);
  ledcSetup(G_channel,pwm_Frequency,pwm_resolution);

  stepper.connectToPins(MOTOR_STEP, MOTOR_DIRECTION);
  stepper.startAsService(1);

  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

//--------------------------------- MAIN CODE ---------------------------------//
void loop() {
  int i = 1;
  RGB_Color(0,255,0);

  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);

  if (digitalRead(Button_White) == HIGH)
    init_acquisition = true;

///////////////////////////////////////////// ROTATION MOVIMENT + ACQUISITION /////////////////////////////////////////////
  if (init_acquisition)
  {
    Serial.println("inicio aquisição");
    RGB_Color(255, 255, 255);
    delay(100);
    do
    {
      digitalWrite(ACQUISITION, 1); //TTL pulse on for acquisition
      delayMicroseconds(50); //pulse length
      digitalWrite(ACQUISITION, 0); //TTL pulse off
      
      // set the speed and acceleration rates for the stepper motor
      stepper.setSpeedInStepsPerSecond(1800);
      stepper.setAccelerationInStepsPerSecondPerSecond(1800);

      stepper.moveRelativeInSteps(400);
      // Rotate the motor in the forward direction one revolution (200 steps). 
      // This function call will not return until the motion is complete.
      //stepper.moveRelativeInSteps(2000);
      //delay(1000);
      // rotate backward 1 rotation, then wait 1 second
      //stepper.moveRelativeInSteps(-200);
      delay(200);

      // This time speedup the motor, turning 10 revolutions.  Note if you
      // tell a stepper motor to go faster than it can, it just stops.
      //stepper.setSpeedInStepsPerSecond(800);
      //stepper.setAccelerationInStepsPerSecondPerSecond(800);
      //stepper.moveRelativeInSteps(200 * 10);
      //delay(2000);

      if (digitalRead(Button_Red) == HIGH)
      {
        init_acquisition = false;
        RGB_Color(255, 0, 0);
        delay(1000);
      }

      if (i == 20)
        init_acquisition = false;
        
      i++;
    }while (init_acquisition);
  }
}

//-------- Function: Define Colors to the Leds --------//
void RGB_Color(int i, int j, int k)
{
  ledcWrite(R_channel,i);
  ledcWrite(G_channel,j);
  ledcWrite(B_channel,k);
}
