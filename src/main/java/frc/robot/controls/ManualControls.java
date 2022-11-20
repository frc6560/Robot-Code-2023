// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.utility.NumberStepper;
import frc.robot.utility.PovNumberStepper;
import frc.robot.Constants;
import frc.robot.utility.Util;

/** Add your docs here. */
public class ManualControls implements DriveCommand.Controls {

    private  XboxController xbox;

    private final PovNumberStepper speed;
    private final PovNumberStepper turnSpeed;

    public ManualControls(XboxController xbox) {
        this.xbox = xbox;

        this.speed = new PovNumberStepper(
            new NumberStepper(Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.4, 0.0, Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.1),
            xbox,
            PovNumberStepper.PovDirection.VERTICAL
        );

        this.turnSpeed = new PovNumberStepper(
            new NumberStepper(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.4, 0.0, Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.1),
            xbox,
            PovNumberStepper.PovDirection.HORIZONTAL
        );
    }

    @Override
    public double driveX() {
        return Util.modifyAxis(xbox.getLeftX() * speed.get());

    }

    @Override
    public double driveY() {
        return Util.modifyAxis(xbox.getLeftY() * speed.get());
    }

    @Override
    public double driveRotation() {
        return Util.modifyAxis(xbox.getRightX() * turnSpeed.get());
    }

    @Override
    public boolean driveResetYaw() {
        return xbox.getStartButton();
    }


    

}
