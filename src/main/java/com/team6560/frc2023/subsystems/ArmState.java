// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

/** Add your docs here. */
public class ArmState {
    private double position;
    private boolean extensionStatus;
    private double clawSpeedMultiplyer;

    public ArmState(double position, boolean extensionStatus, double clawSpeedMultiplyer) {
        this.position = position;
        this.extensionStatus = extensionStatus;
        this.clawSpeedMultiplyer = clawSpeedMultiplyer;
    }

    public double getPosition() {
        return position;
    }

    public boolean getExtentionStatus() {
        return extensionStatus;
    }

    public double getClawSpeedMultiplier() {
        return clawSpeedMultiplyer;
    }
}
