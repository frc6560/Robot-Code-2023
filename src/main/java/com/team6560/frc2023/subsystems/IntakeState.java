// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.team6560.frc2023.subsystems.Arm.ArmPose;

/** Add your docs here. */
public class IntakeState {
    private double position;
    private double suckSpeed;
    private ArmPose armPose;

    public IntakeState(double position, double suckSpeed, ArmPose armPose) {
        this.position = position;
        this.suckSpeed = suckSpeed;
        this.armPose = armPose;
    }

    public double getPosition() {
        return position;
    }

    public double getSuckSpeed() {
        return suckSpeed;
    }

    public ArmPose getArmPose() {
        return armPose;
    }
}
