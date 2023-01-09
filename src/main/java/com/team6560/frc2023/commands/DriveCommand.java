package com.team6560.frc2023.commands;

import com.team6560.frc2023.commands.auto.AutoBuilder;
import com.team6560.frc2023.commands.auto.GoToDoubleSubstationCommand;
import com.team6560.frc2023.commands.auto.GoToSingleSubstationCommand;
import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

        boolean GoToSingleSubstation();
    }

    private Controls controls;

    private AutoBuilder autoBuilder;
    private GoToDoubleSubstationCommand goToDoubleSubstationCommand;
    private GoToSingleSubstationCommand goToSingleSubstationCommand;

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

        this.goToDoubleSubstationCommand = new GoToDoubleSubstationCommand(autoBuilder);
        this.goToSingleSubstationCommand = new GoToSingleSubstationCommand(autoBuilder);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {}

    private boolean goingToDoubleSubstation = false;
    private boolean goingToSingleSubstation = false;
    @Override
    public void execute() {
        if (controls.GoToDoubleSubstation() && !goingToDoubleSubstation) {
            CommandScheduler.getInstance().schedule(goToDoubleSubstationCommand);
            goingToDoubleSubstation = true;
        }
        if (!controls.GoToDoubleSubstation()) goingToDoubleSubstation = false;

        if (controls.GoToSingleSubstation() && !goingToSingleSubstation) {
            CommandScheduler.getInstance().schedule(goToSingleSubstationCommand);
            goingToSingleSubstation = true;
        }
        if (!controls.GoToDoubleSubstation()) goingToSingleSubstation = false;

        
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