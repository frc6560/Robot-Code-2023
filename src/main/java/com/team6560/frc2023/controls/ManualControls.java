// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.controls;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.commands.DriveCommand;
import com.team6560.frc2023.utility.NumberStepper;
import com.team6560.frc2023.utility.PovNumberStepper;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class ManualControls implements DriveCommand.Controls {

    private XboxController xbox;

    private final PovNumberStepper speed;
    private final PovNumberStepper turnSpeed;

    public ManualControls(XboxController xbox) {
        this.xbox = xbox;

        this.speed = new PovNumberStepper(
                new NumberStepper(Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.4, 0.0,
                        Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.05),
                xbox,
                PovNumberStepper.PovDirection.VERTICAL);

        this.turnSpeed = new PovNumberStepper(
                new NumberStepper(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.4, 0.0,
                        Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.05),
                xbox,
                PovNumberStepper.PovDirection.HORIZONTAL);
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
    }
    
    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    @Override
    public double driveX() {
        return modifyAxis(-xbox.getLeftX() * speed.get());

    }

    @Override
    public double driveY() {
        return modifyAxis(xbox.getLeftY() * speed.get());
    }

    @Override
    public double driveRotation() {
        return modifyAxis(-xbox.getRightX() * turnSpeed.get());
    }

    @Override
    public boolean driveResetYaw() {
        return xbox.getStartButton();
    }

}
