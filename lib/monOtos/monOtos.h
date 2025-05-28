#ifndef MONOTOS_H
#define MONOTOS_H

#include <Arduino.h>

// Create an Optical Tracking Odometry Sensor object
QwiicOTOS myOtos;

// Create structs for position, velocity, and acceleration
sfe_otos_pose2d_t posit;
sfe_otos_pose2d_t vel;
sfe_otos_pose2d_t acc;

void Otos_init()
{
    Wire.begin();
    Wire.setClock(100000);
    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
    }
    Serial.println("OTOS connected!");
    Serial.println("Ensure the OTOS is flat and stationary, then enter any key to calibrate the IMU");
    // Perform the self test
    sfTkError_t result = myOtos.selfTest();
    // Check if the self test passed
    if (result == ksfTkErrOk)
    {
        Serial.println("Self test passed!");
    }
    else
    {
        Serial.println("Self test failed!");
    }

    Serial.println("Calibrating IMU...");

    // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
    myOtos.calibrateImu();

    // Set the desired units for linear and angular measurements. Can be either
    // meters or inches for linear, and radians or degrees for angular. If not
    // set, the default is inches and degrees. Note that this setting is not
    // stored in the sensor, it's part of the library, so you need to set at the
    // start of all your programs.
    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    // myOtos.setLinearUnit(kSfeOtosLinearUnitInches);
    myOtos.setAngularUnit(kSfeOtosAngularUnitRadians);
    // myOtos.setAngularUnit(kSfeOtosAngularUnitDegrees);

    // Assuming you've mounted your sensor to a robot and it's not centered,
    // you can specify the offset for the sensor relative to the center of the
    // robot. The units default to inches and degrees, but if you want to use
    // different units, specify them before setting the offset! Note that as of
    // firmware version 1.0, these values will be lost after a power cycle, so
    // you will need to set them each time you power up the sensor. For example, if
    // the sensor is mounted 5 inches to the left (negative X) and 10 inches
    // forward (positive Y) of the center of the robot, and mounted 90 degrees
    // clockwise (negative rotation) from the robot's orientation, the offset
    // would be {-5, 10, -90}. These can be any value, even the angle can be
    // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
    sfe_otos_pose2d_t offset = {-5, 10, -90};
    myOtos.setOffset(offset);

    // Reset the tracking algorithm - this resets the position to the origin,
    // but can also be used to recover from some rare tracking errors
    myOtos.resetTracking();

    // After resetting the tracking, the OTOS will report that the robot is at
    // the origin. If your robot does not start at the origin, or you have
    // another source of location information (eg. vision odometry), you can set
    // the OTOS location to match and it will continue to track from there.
    sfe_otos_pose2d_t currentPosition = {0, 0, 0};
    myOtos.setPosition(currentPosition);

}

void Otos_read()
{       // lecture des données du capteur disponibles dans posit, vel et acc.
    // These values can be read individually like so:
    myOtos.getPosition(posit);
    myOtos.getVelocity(vel);
    myOtos.getAcceleration(acc);

    //// Or burst read them all at once with the following:
    //// myOtos.getPosVelAcc(pos, vel, acc);
//
    //// Print position
    //Serial.println();
    //Serial.println("Position:");
    //Serial.print("X (Meters): ");
    //Serial.println(pos.x);
    //Serial.print("Y (Meters): ");
    //Serial.println(pos.y);
    //Serial.print("Heading (Radians): ");
    //Serial.println(pos.h);
    //
    //// Print velocity
    //Serial.println();
    //Serial.println("Velocity:");
    //Serial.print("X (Meters/sec): ");
    //Serial.println(vel.x);
    //Serial.print("Y (Meters/sec): ");
    //Serial.println(vel.y);
    //Serial.print("Heading (Radians/sec): ");
    //Serial.println(vel.h);
    //
    //// Print acceleration
    //Serial.println();
    //Serial.println("Acceleration:");
    //Serial.print("X (Meters/sec^2): ");
    //Serial.println(acc.x);
    //Serial.print("Y (Meters/sec^2): ");
    //Serial.println(acc.y);
    //Serial.print("Heading (Radians/sec^2): ");
    //Serial.println(acc.h);
//
    // Wait a bit so we don't spam the serial port
    //delay(500);
}

#endif