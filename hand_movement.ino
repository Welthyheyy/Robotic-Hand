#include <Servo.h>
#include <AccelStepper.h>

AccelStepper stepper(
  AccelStepper::HALF4WIRE,
  2, 4, 3, 5
);

Servo indexServ;
Servo middleServ;
Servo thumbServ;
Servo ringPinkyServ;


int indexPin = 9;
int middlePin = 10;
int thumbPin = 6;
int ringPinkyPin = 11;

int targetServoAngle = 90;
long targetStepperPos = 0;

void setup() {
  Serial.begin(9600);

  indexServ.attach(indexPin);
  middleServ.attach(middlePin);
  thumbServ.attach(thumbPin);
  ringPinkyServ.attach(ringPinkyPin);

  indexServ.write(targetServoAngle);
  middleServ.write(targetServoAngle);
  thumbServ.write(targetServoAngle);
  ringPinkyServ.write(targetServoAngle);
  
  
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(600);
  stepper.setAcceleration(300);

}

void loop() {
  //  Always run stepper
  stepper.run();

  //  Handle serial without blocking
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
 
    if (cmd.startsWith("I:")) {
      targetServoAngle = constrain(cmd.substring(2).toInt(), 0, 180);
      indexServ.write(targetServoAngle);
    }

    if (cmd.startsWith("M:")) {
      targetServoAngle = constrain(cmd.substring(2).toInt(), 0, 180);
      middleServ.write(targetServoAngle);
    }

    if (cmd.startsWith("T:")) {
      targetServoAngle = constrain(cmd.substring(2).toInt(), 0, 180);
      thumbServ.write(targetServoAngle);
    }

    if (cmd.startsWith("P:")) {
      targetServoAngle = constrain(cmd.substring(2).toInt(), 0, 180);
      ringPinkyServ.write(targetServoAngle);
    }

    if (cmd.startsWith("W:")) {
      targetStepperPos = cmd.substring(2).toInt();  
      stepper.moveTo(targetStepperPos);             
    }
  }
}
