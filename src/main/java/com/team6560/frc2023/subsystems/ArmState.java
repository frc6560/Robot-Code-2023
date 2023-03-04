// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**

The ArmState class represents the current state of the arm subsystem.
It contains the current position, extension status, and claw speed multiplier of the arm.
The arm subsystem is responsible for controlling the movement and position of the robot's arm.
This class can be modified to add additional functionality as needed.
*/
package com.team6560.frc2023.subsystems;

/** Add your docs here. */
public class ArmState {
    private double position; // The current position of the arm
    private boolean extensionStatus; // Whether the arm is extended or retracted
    private double clawSpeedMultiplyer; // The speed multiplier for the claw mechanism

    /**
    * Creates a new ArmState object with the given parameters.
    * 
    * @param position The current position of the arm.
    * @param extensionStatus Whether the arm is extended or retracted.
    * @param clawSpeedMultiplyer The speed multiplier for the claw mechanism.
    */
    public ArmState(double position, boolean extensionStatus, double clawSpeedMultiplyer) {
        this.position = position;
        this.extensionStatus = extensionStatus;
        this.clawSpeedMultiplyer = clawSpeedMultiplyer;
    }

    /**
    * Returns the current position of the arm.
    * 
    * @return The current position of the arm.
    */
    public double getPosition() {
        return position;
    }

    /**
    * Returns whether the arm is extended or retracted.
    * 
    * @return True if the arm is extended, false if the arm is retracted.
    */
    public boolean getExtentionStatus() {
        return extensionStatus;
    }

    /**
    * Returns the speed multiplier for the claw mechanism.
    * 
    * @return The speed multiplier for the claw mechanism.
    */
    public double getClawSpeedMultiplier() {
        return clawSpeedMultiplyer;
    }
}
