#include <Wire.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>

#define LightThreshold 500
#define CentralAllignDur 200
#define InstructionDuration 200000000
#define TurnDuration 500

ZumoBuzzer buzzer;
//ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

// This is the maximum speed the motors will be allowed to turn.
// (400 lets the motors go at top speed; decrease to impose a speed limit)
const int MAX_SPEED = 200;
int LeftMotSpeed =0;
int RightMotSpeed = 0;
bool MoveZumo =false;
float InstructionEnd=0;
bool ZumoMoving =false;
int NumFrontSensorsTrigged =0;
int LightAdjustments[]= {0,-100,-100,-100,-100,0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Please Press Button on Zumo");
  button.waitForButton();
  Serial.println("Button press acknowledged");
  delay(500);

}

void loop() {
  // put your main code here, to run repeatedly:
    //Serial.println("Please Provide a Direction");
    char direction = (char) Serial.read();
    switch(direction){
            case 'w': case 'W': 
               LeftMotSpeed =MAX_SPEED;
               RightMotSpeed = MAX_SPEED;
               MoveZumo = true;
               InstructionEnd = millis()+InstructionDuration;
            break;
            case 's': case 'S': 
               LeftMotSpeed =-MAX_SPEED;
               RightMotSpeed =- MAX_SPEED;
               MoveZumo = true;
               InstructionEnd = millis()+InstructionDuration;
            break;
            case 'a': case 'A': 
               LeftMotSpeed =-MAX_SPEED;
               RightMotSpeed = MAX_SPEED;
               MoveZumo = true;
               InstructionEnd = millis()+TurnDuration;
            break;
            case 'd': case 'D': 
               LeftMotSpeed =MAX_SPEED;
               RightMotSpeed =-MAX_SPEED;
               MoveZumo = true;
               InstructionEnd = millis()+TurnDuration;
             break;
             case 'q': case 'Q': 
              Halt();

    }
    sensors.read(sensor_values); 
    if (InstructionEnd<millis() && MoveZumo == true)
    {
      Halt();
      Serial.println("Zumo Moving Completed");
    }
    //front collission
    int TriggeredSensors;
    //side collissions
    //left
    if (sensor_values[0]>LightThreshold)
    {
      int TimeCapt = millis();
      delay(50);  //wait 100ms for more sensors to move onto wall
      motors.setSpeeds(0,0);
      TriggeredSensors = CheckForEndOfCorridor();  
      Serial.println(TriggeredSensors);
      if (TriggeredSensors>=4)
      {
            EndOfCorridor();
      }
      else 
      {
        Serial.println("Oops hit the left edge");
        motors.setSpeeds(-MAX_SPEED,-MAX_SPEED);
        delay(300);
        motors.setSpeeds(MAX_SPEED,0);
        int Duration = TriggeredSensors*2;
        delay(200*Duration);
      }
      TimeCapt = millis()-TimeCapt;
      InstructionEnd+=TimeCapt;
    }
    //right
    else if (sensor_values[5]>LightThreshold)
    {
      int TimeCapt = millis();
      delay(50);
      motors.setSpeeds(0,0);
      delay(300);
      TriggeredSensors=CheckForEndOfCorridor();
      Serial.println(TriggeredSensors);
      if (TriggeredSensors>=4)
      {
        EndOfCorridor();
      }
      else
      {
      Serial.println("Oops hit the right edge");
      motors.setSpeeds(-MAX_SPEED,-MAX_SPEED);
      delay(300);
      motors.setSpeeds(0,MAX_SPEED);
      int Duration = TriggeredSensors*2;
      delay(200*Duration);
      }
      TimeCapt = millis()-TimeCapt;
      InstructionEnd+=TimeCapt;
    }
    NumFrontSensorsTrigged =0;
    motors.setSpeeds(LeftMotSpeed, RightMotSpeed);
}

void Halt()
{
  LeftMotSpeed =0;
  RightMotSpeed = 0;
  MoveZumo=false;
  Serial.println("Robot Stopped");
  delay(300);
  Serial.println("Please provide new instruction");
}

int CheckForEndOfCorridor()
{
  sensors.read(sensor_values); 
  int SensorsTriggered =0;
   for (int s =0; s<NUM_SENSORS; s++)
    {
      Serial.println(sensor_values[s]);
      if (sensor_values[s]>LightThreshold+LightAdjustments[s])
      {
        SensorsTriggered++;
      }
    }
    Serial.println("Readings Complete");
    return SensorsTriggered;
}

void EndOfCorridor()
{
     Serial.println("End of Corridor encountered! Stopping movement");   
     motors.setSpeeds(-MAX_SPEED,-MAX_SPEED);
     delay(150);
     Halt();
}

