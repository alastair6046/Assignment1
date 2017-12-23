#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>

ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
int lastError=0;
const int MaxSpeed = 200;


void setup() {
  // put your setup code here, to run once:
  reflectanceSenors.init();
  pinMode(13,OUTPUT);
  digitalWrite(13, HIGH);

  delay(1000);
//  int i;
//  for(i = 0; i < 80; i++)
//  {
 //   if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
//      motors.setSpeeds(-200, 200);
//    else
//      motors.setSpeeds(200, -200);
//    reflectanceSensors.calibrate();
//
//    // Since our counter runs to 80, the total delay will be
//    // 80*20 = 1600 ms.
//    delay(20);
  //}
  //motors.setSpeeds(0,0);
 //digitalWrite(13, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
boolean sawLine = false ;
setMotors ( MaxSpeed , MaxSpeed );
while (distance traveled <10) {
delay (50);
if (sensor sees line) {
sawLine = true ;
}
}

}
