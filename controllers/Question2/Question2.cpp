// File:          Question2.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#define TIMESTEP 64
#define MAX_SPEED 6.28

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Motor *left_motor = robot->getMotor("left wheel motor");
  Motor *right_motor = robot->getMotor("right wheel motor");
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);

  //Distance Sensor
  DistanceSensor *ds_left = robot->getDistanceSensor("ds_left");
  DistanceSensor *ds_right = robot->getDistanceSensor("ds_right");
  DistanceSensor *ds_top = robot->getDistanceSensor("ds_top");
  DistanceSensor *ds_bottom = robot->getDistanceSensor("ds_bottom");
  ds_left->enable(TIMESTEP);
  ds_right->enable(TIMESTEP);
  ds_top->enable(TIMESTEP);
  ds_bottom->enable(TIMESTEP);
  

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIMESTEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    double ds_left_val = ds_left->getValue();
    double ds_right_val = ds_right->getValue();
    double ds_top_val = ds_top->getValue();
    double ds_bottom_val = ds_bottom->getValue();
    
    // If  ds value is < 100 there is a collision
    std::cout << "Left : "<< ds_left_val << " Right :" << ds_right_val << "Top :" << ds_top_val << "Bottom :" << ds_bottom_val << std::endl;
    // Process sensor data here.
    left_motor->setVelocity(MAX_SPEED);
    right_motor->setVelocity(MAX_SPEED);
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
