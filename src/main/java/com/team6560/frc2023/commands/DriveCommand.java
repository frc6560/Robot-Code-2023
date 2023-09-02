package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private final Drivetrain drivetrain;


    public static interface Controls {
        double driveX();

        double driveY();

        double driveRotationX();

        double driveRotationY();

        boolean driveResetYaw();

        boolean driveResetGlobalPose();
    }

    private Controls controls;

    
    public DriveCommand(Drivetrain drivetrainSubsystem, Controls controls) {
        this.drivetrain = drivetrainSubsystem;
        this.controls = controls;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (controls.driveResetYaw()) {
            drivetrain.zeroGyroscope();
        }

        if (controls.driveResetGlobalPose()){
            drivetrain.resetOdometry(new Pose2d());
        }


        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        controls.driveX(),
                        controls.driveY(),
                        controls.driveRotationX(),
                        drivetrain.getGyroscopeRotationNoApriltags())); // perhaps use getRawGyroRotation() instead?
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}
