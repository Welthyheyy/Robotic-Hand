#include <Servo.h>
#include <AccelStepper.h>

AccelStepper stepper(
  AccelStepper::HALF4WIRE,
  2, 4, 3, 5
);

Servo indexServ;

int indexPin = 9;

int targetServoAngle = 90;
long targetStepperPos = 0;

void setup() {
  Serial.begin(9600);

  indexServ.attach(indexPin);
  indexServ.write(targetServoAngle);

  stepper.setMaxSpeed(600);
  stepper.setAcceleration(300);
}

void loop() {
  // 🔹 Always run stepper
  stepper.run();

  // 🔹 Handle serial without blocking
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    if (cmd.startsWith("S:")) {
      targetServoAngle = constrain(cmd.substring(2).toInt(), 0, 180);
      indexServ.write(targetServoAngle);
    }

    if (cmd.startsWith("W:")) {
      int delta = cmd.substring(2).toInt();
      delta = constrain(delta, -20, 20);
      targetStepperPos += delta;
      stepper.moveTo(targetStepperPos);
    }
  }
}
