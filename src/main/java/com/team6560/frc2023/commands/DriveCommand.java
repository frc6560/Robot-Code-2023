package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
    private final Drivetrain drivetrain;

    /**
     * Interface for defining the controls for the drive command.
     */
    public static interface Controls {
        double driveX();

        double driveY();

        double driveRotationX();

        double driveRotationY();

        boolean driveResetYaw();

        boolean driveResetGlobalPose();
    }

    private Controls controls;

    private PIDController driveRotationPIDController = new PIDController(0.06, 0.05, 0.0);
    private PIDController driveTranslationYPIDController = new PIDController(0.13, 0.02, 0.0);
    private PIDController driveTranslationXPIDController = new PIDController(0.13, 0.02, 0.0);



    /**
     * Creates a new `DriveCommand` instance.
     *
     * @param drivetrainSubsystem the `Drivetrain` subsystem used by the command
     * @param controls            the controls for the command
     */
    public DriveCommand(Drivetrain drivetrainSubsystem, Controls controls) {
        this.drivetrain = drivetrainSubsystem;
        this.controls = controls;

        addRequirements(drivetrainSubsystem);

        driveRotationPIDController.setIntegratorRange(-0.5, 0.5);
        // driveRotationPIDController.setTolerance(3.5);
        driveRotationPIDController.setTolerance(0.0);
        driveRotationPIDController.enableContinuousInput(-180.0, 180.0);

        driveTranslationYPIDController.setIntegratorRange(-0.6, 0.6);
        driveTranslationYPIDController.setTolerance(0.0);
        driveTranslationYPIDController.enableContinuousInput(-180.0, 180.0);

        driveTranslationXPIDController.setIntegratorRange(-0.6, 0.6);
        driveTranslationXPIDController.setTolerance(0.0);
        driveTranslationXPIDController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (controls.driveResetYaw()) {
            drivetrain.zeroGyroscope();
        }

        if (controls.driveResetGlobalPose())
            drivetrain.resetOdometry(new Pose2d());


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
