package com.team6560.frc2023.commands;

import com.team6560.frc2023.commands.auto.AutoBuilder;
import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private final Drivetrain drivetrain;

    /**
     * Interface for defining the controls for the drive command.
     */
    public static interface Controls {
        double driveX();

        double driveY();

        double driveRotation();

        boolean driveResetYaw();

        boolean GoToDoubleSubstation();

        boolean driveResetGlobalPose();
    }

    private Controls controls;

    private AutoBuilder autoBuilder;

    private boolean goingToPose = false;
    private Command goToPoseAutoCommand;

    /**
     * Creates a new `DriveCommand` instance.
     *
     * @param drivetrainSubsystem the `Drivetrain` subsystem used by the command
     * @param controls            the controls for the command
     */
    public DriveCommand(Drivetrain drivetrainSubsystem, AutoBuilder autoBuilder, Controls controls) {
        this.drivetrain = drivetrainSubsystem;
        this.autoBuilder = autoBuilder;
        this.controls = controls;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        if (controls.GoToDoubleSubstation() && !goingToPose) {
            goToPoseAutoCommand = autoBuilder.goToPose(new Pose2d());
            goToPoseAutoCommand.initialize();
            goingToPose = true;

        }
        if (goingToPose && goToPoseAutoCommand != null && !goToPoseAutoCommand.isFinished()) {
            goToPoseAutoCommand.execute();
            return;
        }
        goingToPose=false;

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        controls.driveX(),
                        controls.driveY(),
                        controls.driveRotation(),
                        drivetrain.getGyroscopeRotation()));
        if (controls.driveResetYaw()) {
            drivetrain.zeroGyroscope();
        }

        if (controls.driveResetGlobalPose()) drivetrain.resetOdometry(new Pose2d());
    }

    /**
     * Called when the command ends or is interrupted.
     *
     * @param interrupted whether the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopModules();
    }
}