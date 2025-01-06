// Bounce.pde
// -*- mode: C++ -*-
//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <FABRIK2D.h>

// Define a stepper and the pins it will use
//AccelStepper upperStepper(1, 2, 5);  // x on board, pin 2 = step, pin 5 = direction
AccelStepper upperStepper(1, 4, 7);  // z on board pin 4 = step, pin 7 = direction
AccelStepper lowerStepper(1, 3, 6);  // y on board, pin 3 = step, pin 6 = direction
MultiStepper steppers;

const byte enablePin = 8;
const int maxSpeed = 100;
const int acceleration = 40;

const byte numChars = 128;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;

// A 2DOF arm, where we have 2 links and 2+1 joints, 
// where the end effector counts as one joint in this case.
int lengths[] = {194, 184}; // Length of shoulder and elbow in mm.
Fabrik2D fabrik2D(3, lengths); // 3 Joints in total

void setup() {
  // enable stepper board
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  // initialize serial:
  Serial.begin(9600);

  // Change these to suit your stepper if you want
  upperStepper.setMaxSpeed(maxSpeed);
  upperStepper.setAcceleration(acceleration);
  steppers.addStepper(upperStepper);

  lowerStepper.setMaxSpeed(maxSpeed);
  lowerStepper.setAcceleration(acceleration);
  steppers.addStepper(lowerStepper);

  fabrik2D.setTolerance(0.5);

  //upperStepper.moveTo(50);
  //lowerStepper.moveTo(50);
}

void loop() {
  recvWithEndMarker();
  if (newData == true) {
    executeCommand(String(receivedChars));
    newData = false;
  }
  //upperStepper.run();
  //lowerStepper.run();
  steppers.run();
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

long upperDegreesToSteps(double degrees) {
  return (long) (degrees / 90.0 * 100.0 * -1.);
}

long lowerDegreesToSteps(double degrees) {
  return (long) (degrees / 90.0 * 100.0 * -1.);
}

enum CommandStage {
  COMMAND_LETTER,
  COMMAND_NUMBER,
  AXIS,
  AXIS_VALUE,
};

void executeCommand(String command) {
  CommandStage stage = COMMAND_LETTER;

  int startIndex = 0;
  char axis = '0';

  // TODO floats
  int xTarget = 0;
  int yTarget = 0;

  for(std::string::size_type i = 0; i < command.length(); ++i) {
    switch (stage) {
      case COMMAND_LETTER:
        if (command[i] == 'G') {
          // set command letter
          startIndex = i + 1;
          stage = COMMAND_NUMBER;
        } else {
          // BAD THING
          Serial.print("Bad command letter: ");
          Serial.print(command[i]);
          Serial.println(".");
          return;
        }
        break;
      case COMMAND_NUMBER:
        if (command[i] == ' ') {
          //int commandNumber = command.substring(startIndex, i).toInt();
          stage = AXIS;
        }
        break;
      case AXIS:
        axis = command[i];
        startIndex = i + 1;
        stage = AXIS_VALUE;
        break;
      case AXIS_VALUE:
        if (command[i] == ' ') {
          if (axis == 'x' || axis == 'X') {
            xTarget = command.substring(startIndex, i).toInt();
            stage = AXIS;
            break;
          } else if (axis == 'y' || axis == 'Y') {
            yTarget = command.substring(startIndex, i).toInt();
            stage = AXIS;
            break;
          } else {
            // BAD THING
            Serial.print("Bad axis letter: ");
            Serial.print(axis);
            Serial.println(".");
            return;
          }
        }
        break;
    }
  }
  if (stage == AXIS_VALUE) {
          if (axis == 'x' || axis == 'X') {
            xTarget = command.substring(startIndex).toInt();
          } else if (axis == 'y' || axis == 'Y') {
            yTarget = command.substring(startIndex).toInt();
          } else {
            // BAD THING
            Serial.print("Bad axis letter: ");
            Serial.print(axis);
            Serial.println(".");
            return;
          }
  }

  Serial.print("Going to (x, y): (");
  Serial.print(xTarget);
  Serial.print(", ");
  Serial.print(yTarget);
  Serial.println(")");

  fabrik2D.solve(xTarget, yTarget, lengths);

  double shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG; // In degrees
  double elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG; // In degrees

  Serial.print("Shoulder angle: ");
  Serial.println(shoulderAngle);
  Serial.print("Elbow angle: ");
  Serial.println(elbowAngle);

  long positions[2];
  positions[0] = upperDegreesToSteps(shoulderAngle);
  positions[1] = lowerDegreesToSteps(elbowAngle);

  Serial.print("Moving steppers to: ");
  Serial.print(positions[0]);
  Serial.print(", ");
  Serial.println(positions[1]);

  steppers.moveTo(positions);
}
