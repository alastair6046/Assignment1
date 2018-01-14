#include <Wire.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <LSM303.h>
#include <NewPing.h>

//#define LightThreshold 500
#define CentralAllignDur 200
#define InstructionDuration 200000000  //Used for testing, decreased for when I dont want to robot to wonder off
#define TurnDuration 500  //Duration of a turn
#define TurnSpeed 200
#define BaseTurnSpeed 100

#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate
#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define DEVIATION_THRESHOLD 5

#define TriggerPin 6
#define EchoPin 2
#define MaxDistance 200
#define NUM_SENSORS 6  // theres 6 sensors on the robot

#define ObjectDetectionRange 15
#define USSamples 13

NewPing sonar(TriggerPin, EchoPin, MaxDistance); //set up the UltraSonic sensor
ZumoBuzzer buzzer;
//ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
LSM303 compass;
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

unsigned int sensor_values[NUM_SENSORS]; //array to store the values of the light sensors
unsigned int SensorLightThresholds[NUM_SENSORS]; //array to store the treshold value of the light sensors.  Set in Setup


// This is the maximum speed the motors will be allowed to turn.
// (400 lets the motors go at top speed; decrease to impose a speed limit)
const int MAX_SPEED = 200;  //max speed of robot
int LeftMotSpeed = 0; //left motor speed
int RightMotSpeed = 0; //right motor speed
bool MoveZumo = false; //true when Zumo is moving
float InstructionEnd = 0;
bool ZumoMoving = false;
int NumFrontSensorsTrigged = 0;
int RoomNumber = 1;
//int LightAdjustments[]= {0,-100,-100,-100,-100,0};
bool ObjectIdentified;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Please Press Button on Zumo to begin setup");  //Ask user to press button on zumo to begin
  button.waitForButton(); //waiting for button press
  Serial.println("Please move the front of the zuomo on top of the wall then press the button again"); //ask user to move the light sensors of the zumo over a piece of wall
  button.waitForButton();
  Serial.println("Button Press Acknowledged... processing");
  CalibrateLightSensors();
  Serial.println("Please Move Zumo to beginning then press button again to calibrate compass");
  button.waitForButton();
  Serial.println("Calibrating Compass");
  CalibrateCompass();
  Serial.println("Compass Calibration Complete");
  delay(500);
  Serial.println("Zumo Setup Complete.  Proceed when ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Please Provide a Direction");
  char direction = (char) Serial.read(); //wait for user to input a direct
  int turnSize;// = (int) Serial.parseInt();
  //  Serial.print("button press = ");
  //  Serial.println(direction);
  //Serial.print("Turn Size = ");
  // Serial.println(turnSize);
  switch (direction) {
    case 'w': case 'W': //forwards
      LeftMotSpeed = MAX_SPEED; //Full Speed ahead!!!
      RightMotSpeed = MAX_SPEED;
      MoveZumo = true; //Zumo is now moving
      InstructionEnd = millis() + InstructionDuration; //for testing.  Sets millis time for when Zumo should stop moving
      break;
    case 's': case 'S': //stop
      Halt();  //Stop everything
      break;
    case 'a': case 'A': //rotate left
      // LeftMotSpeed =-MAX_SPEED;  //turn left on the spot roughly 90 degrees
      // RightMotSpeed = MAX_SPEED;
      // MoveZumo = true; //Zumo is now moving
      InstructionEnd = millis() + TurnDuration; //duration of turn maneuvour
      TurnZumo('l', turnSize);
      break;
    case 'd': case 'D': //rotate right
      //   LeftMotSpeed =MAX_SPEED;  //turn right on the spot roughly 90 degrees
      //   RightMotSpeed =-MAX_SPEED;
      //   MoveZumo = true;  //Zumo is now moving
      InstructionEnd = millis() + TurnDuration; //duration of turn maneuvour
      TurnZumo('r', turnSize);
      break;
    case 'q': case 'Q': //room on left
      Serial.println("Room on left registered");
      RoomExplore('l');
      break;
    case 'e': case 'E': //room on right
      Serial.println("Room on right registered");
      RoomExplore('r');
      break;


  }
  //Check US sensor for collissions
  if (sonar.ping_cm() < ObjectDetectionRange && sonar.ping_cm()!=0)
  {
    Serial.print(sonar.ping_cm());
    Serial.println(" Object Detected in front of robot."); 
    Halt();
    motors.setSpeeds(-MAX_SPEED, -MAX_SPEED); //move away from object
    delay(100);
    motors.setSpeeds(0,0);
  }
  
  sensors.read(sensor_values); //every loop take a new sensor reading
  if (InstructionEnd < millis() && MoveZumo == true) //if the Instruction end is less than the duration that Zumo has been running for halt.
  {
    Halt();
    Serial.println("Zumo Moving Completed"); //tell user instructions have timed out
  }
  int TriggeredSensors; //number of sensors triggered
  //side collissions
  //left
  if (sensor_values[0] > SensorLightThresholds[0]) //if left most sensor if triggered
  {
    int TimeCapt = millis();  //store time of collision
    delay(20);  //wait 20ms for more sensors to move onto wall
    motors.setSpeeds(0, 0); //stop
    TriggeredSensors = CheckForEndOfCorridor();  //check how many sensors have been triggered.
    Serial.println(TriggeredSensors);  //debug to report how many sensors have triggered
    if (TriggeredSensors >= 3) //if 3 or more sensors have triggered it means the end of the corridor has been reached
    {
      EndOfCorridor();  //commence end of corridor instructions
    }
    else //else Zumo has hit a side wall
    {
      Serial.println("Oops hit the left edge");  //report a collission
      motors.setSpeeds(-MAX_SPEED, -MAX_SPEED); //reverse
      delay(300); //for 300ms
      motors.setSpeeds(MAX_SPEED, 0); //turn to straighten
      delay(200 * TriggeredSensors); //Spend time turning based on how many sensors triggered.
    }
    TimeCapt = millis() - TimeCapt; //work out duration of reverse maneuvour
    InstructionEnd += TimeCapt; //add duration of reverse maneuvour to Instruction End, so it doesnt effect the distance travelled forwards
  }
  //right
  else if (sensor_values[5] > SensorLightThresholds[5])
  {
    int TimeCapt = millis(); //store time of collission
    delay(20); //wait 20ms for more snesors to move onto wall
    motors.setSpeeds(0, 0); //stop
    TriggeredSensors = CheckForEndOfCorridor(); //check how many sensors have been triggered
    Serial.println(TriggeredSensors); //debug to report how many sensors have been triggered
    if (TriggeredSensors >= 3) //if 3 or more sensors have triggered it means the end of the corridor has been reached
    {
      EndOfCorridor(); //commence end of corridor instructions
    }
    else
    {
      Serial.println("Oops hit the right edge"); //report a collission
      motors.setSpeeds(-MAX_SPEED, -MAX_SPEED); //reverse
      delay(150); //for 300ms
      motors.setSpeeds(0, MAX_SPEED); //turn to straighten
      delay(200 * TriggeredSensors); //Spend time turning based on how many sensors have triggered
    }
    TimeCapt = millis() - TimeCapt; //work out duration of reverse maneuvour
    InstructionEnd += TimeCapt; //add duration of reverse maneuvour to Instruction End, so it doesnt effect the distance travelled forwards
  }
  NumFrontSensorsTrigged = 0; //reset value of NumFrontSensors triggered;
  motors.setSpeeds(LeftMotSpeed, RightMotSpeed); //set the speed of the motors
}

void Halt()
{
  motors.setSpeeds(0,0);
  LeftMotSpeed = 0; //stop left motor
  RightMotSpeed = 0; //stop right motor
  MoveZumo = false; //zumo no longer moving
  Serial.println("Robot Stopped"); //tell user Zumo has stopped
  delay(300); //wait 300ms
  //Serial.println("Please provide new instruction"); //ask for a new instruction
}

int CheckForEndOfCorridor()
{
  sensors.read(sensor_values); //take a new sensor reading now that zumo has moved a small amount
  int SensorsTriggered = 0; //Number of Sensors triggered
  for (int s = 0; s < NUM_SENSORS; s++) //for each sensor
  {
    //Serial.println(sensor_values[s]); //debug the value of the sensor
    if (sensor_values[s] > SensorLightThresholds[s]) //if sensor is above the Light Threshold
    {
      SensorsTriggered++; //Increment # of sensors triggered
    }
  }
  //  button.waitForButton();  //debug freeze for wall colliding testing
 // Serial.println("Readings Complete"); //debug that reading is complete
  return SensorsTriggered; //return the number of sensors that have been triggered
}

void EndOfCorridor()
{
  Serial.println("End of Corridor encountered! Stopping movement");   //inform user the end of the corridor has been reached
  motors.setSpeeds(-MAX_SPEED, -MAX_SPEED); //reverse to move off the wall
  delay(100); //for 100ms
  Halt(); //Stop all movement
}

void RoomExplore(char direction)
{
  float RoomLength;
  
  Serial.print("Room Exploring started. Entering room # ");
  Serial.print(RoomNumber);
  Serial.print( "On the ");
  if (direction == 'l')
  {
    Serial.println("left.");
  }
  else if (direction == 'r')
  {
    Serial.println("right.");
  }
  
  TurnZumo(direction, 90);
  RoomLength = millis();
  motors.setSpeeds(MAX_SPEED, MAX_SPEED);
  sensors.read(sensor_values);
  int loopNumber =0;
  while (sensor_values[5] < SensorLightThresholds[5] &&  sensor_values[0] < SensorLightThresholds[0])
  {
    ++loopNumber;
    //Serial.print("Loop Number ");
    //Serial.println(loopNumber);
    sensors.read(sensor_values);
  }
  RoomLength = millis() - RoomLength;
  motors.setSpeeds(-MAX_SPEED, -MAX_SPEED);
  delay(RoomLength/2);
  motors.setSpeeds(0, 0);
  delay(50);
  int Pings [USSamples];
  int NumOfObjectsDetected =0;
  for (int i = 0; i < USSamples; i++)
  {
    TurnZumo('r', 30);
    Serial.print("Ping: ");
    Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
    Serial.println("cm");
    Pings[i] = sonar.ping_cm();
//
    if (Pings[i] <ObjectDetectionRange && Pings[i] !=0 )
    {
      NumOfObjectsDetected++;
    }
  }
  Serial.print(NumOfObjectsDetected);
  Serial.print(" Object/s detected within room #");
  Serial.println(RoomNumber);
  ++ RoomNumber;

}

void CalibrateCompass()
{
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32767, -32767, -32767};
  unsigned char index;
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

  motors.setSpeeds(TurnSpeed, -TurnSpeed);
  for (index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    //Serial.println(index);

    delay(50);
  }
  motors.setSpeeds(-TurnSpeed, TurnSpeed);
  for (index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

   // Serial.println(index);

    delay(50);
  }
  motors.setSpeeds(0, 0);
  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;
}

void TurnZumo(char turnDirection, int magnitude)
{
  motors.setSpeeds(0, 0);
  delay(100);
  float heading, relative_heading;
  int speed;
  int directionFactor;
  static float target_heading = averageHeading();
  //Serial.printu("Turning Magnitude");
  //Serial.println(magnitude);
  if (magnitude == 0)
  {
    magnitude = 90; //if no magnitude is set. Set magnitude to 90
  }
  // Heading is given in degrees away from the magnetic vector, increasing clockwise
  heading = averageHeading();  //set heading to current heading of Zumo
  if (turnDirection == 'r')
  {
    target_heading = fmod(averageHeading() + magnitude, 360); //heading of target
  }
  else if (turnDirection == 'l')
  {
    target_heading = fmod(averageHeading() - magnitude, 360); //heading of target
  }
  relative_heading = relativeHeading(heading, target_heading);
  //  Serial.print("Relative Heading =");
  //  Serial.println(relative_heading);
  //  Serial.print("Target Heading");
  //  Serial.println(target_heading);
  //  Serial.print("Current Heading");
  //  Serial.println(heading);
  while (abs(relative_heading) > DEVIATION_THRESHOLD)
  {
    //    Serial.print("Relative Heading =");
    //    Serial.println(relative_heading);
    //    Serial.print("Target Heading");
    //    Serial.println(target_heading);
    //    Serial.print("Current Heading");
    //    Serial.println(heading);

    heading = averageHeading();

    relative_heading = relativeHeading(heading, target_heading);
    speed = TurnSpeed * relative_heading / 180;

    if (speed < 0)
    { speed -= BaseTurnSpeed;
    }
    else
    {
      speed += BaseTurnSpeed;
    }
    // speed = speed*directionFactor;
    // Serial.print("The Speed for turning right is ");
    //Serial.println(speed);
    motors.setSpeeds(speed, -speed);
    // Serial.print("Turning");
  }
}

template <typename T> float heading(LSM303::vector<T> v) //function from compass example
{
  float x_scaled =  2.0 * (float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0 * (float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled) * 180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to) //function from compass example
{
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading() //function from compass example
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for (int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  //  Serial.println(heading(avg));
  return heading(avg);
}

void CalibrateLightSensors()
{
  delay(300);  //delay so pressing on button doesnt interfere with sensor readings
  unsigned int SensorLightTotal[NUM_SENSORS] = {0, 0, 0, 0, 0, 0};

  for ( int j = 0; j < 10; j++)
  {
    delay(200);
    sensors.read(SensorLightThresholds); //take a light sensor reading
    for (int i = 0; i < NUM_SENSORS; i++) //for each sensor
    {
      SensorLightTotal[i] += SensorLightThresholds[i];
    }
  }
  for (int k = 0; k < NUM_SENSORS; k++)
  {
    SensorLightThresholds[k] = SensorLightTotal[k] / 10 * 0.95;
    SensorLightThresholds[k] = 300;
  }

}

