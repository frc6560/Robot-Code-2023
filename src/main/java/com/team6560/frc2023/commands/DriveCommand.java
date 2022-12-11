package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;

    /**
     * Interface for defining the controls for the drive command.
     */
    public static interface Controls {
        double driveX();

        double driveY();

        double driveRotation();

        boolean driveResetYaw();
    }

    private Controls controls;

    /**
     * Creates a new `DriveCommand` instance.
     *
     * @param drivetrainSubsystem the `Drivetrain` subsystem used by the command
     * @param controls            the controls for the command
     */
    public DriveCommand(Drivetrain drivetrainSubsystem, Controls controls) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.controls = controls;

        addRequirements(drivetrainSubsystem);
    }

    
    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        controls.driveX(),
                        controls.driveY(),
                        controls.driveRotation(),
                        m_drivetrainSubsystem.getGyroscopeRotation()));
        if (controls.driveResetYaw()) {
            m_drivetrainSubsystem.zeroGyroscope();
        }

    }

    /**
     * Called when the command ends or is interrupted.
     *
     * @param interrupted whether the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopModules();
    }
}