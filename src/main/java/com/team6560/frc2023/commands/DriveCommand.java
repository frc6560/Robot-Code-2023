package com.team6560.frc2023.commands;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.commands.auto.AutoBuilder;
import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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

        boolean overrideMaxVisionPoseCorrection();

        boolean driveIsClimbing();

        double climbVelocityL();

        double climbVelocityR();
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
            goToPoseAutoCommand = autoBuilder.goToPose(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0)));
            goToPoseAutoCommand.initialize();
            goingToPose = true;

        }
        if (goingToPose && goToPoseAutoCommand != null && !goToPoseAutoCommand.isFinished()) {
            goToPoseAutoCommand.execute();
            return;
        }
        goingToPose=false;

        if (controls.driveResetYaw()) {
            drivetrain.zeroGyroscope();
        }

        if (controls.driveResetGlobalPose()) drivetrain.resetOdometry(new Pose2d());

        drivetrain.setOverrideMaxVisionPoseCorrection(controls.overrideMaxVisionPoseCorrection());



        setClimbExtension(controls.driveIsClimbing());

        drivetrain.setBatteryBullshit(controls.driveIsClimbing());

        if (controls.driveIsClimbing()) {
            
            drivetrain.setChassisState(Constants.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, controls.driveY(),0.0)));

            // drivetrain.setLeftClimbExtensionVelocity(controls.climbVelocityL());

            // drivetrain.setRightClimbExtensionVelocity(controls.climbVelocityR());

            //drivetrain wheel radius 2in
            //climb wheel radius 1.75in

            drivetrain.setClimbDriveMotorVelocity(Units.radiansPerSecondToRotationsPerMinute(drivetrain.getAverageModuleDriveAngularTangentialSpeed() / Units.inchesToMeters(0.7)));

        } else {
            drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        controls.driveX(),
                        controls.driveY(),
                        controls.driveRotation(),
                        drivetrain.getGyroscopeRotation())); // perhaps use getRawGyroRotation() instead?
        }
        
    }

    public void setClimbExtension(boolean extended) {
        double leftCurrPose = drivetrain.getLeftClimbPosition();
        double rightCurrPose = drivetrain.getRightClimbPosition();

        double target = extended ? 135.0 : 0.0;

        
        if (Math.abs(leftCurrPose - target) > 5.0)
            drivetrain.setLeftClimbExtensionVelocity(Math.copySign(0.12,  target - leftCurrPose));
        else if (Math.abs(leftCurrPose - target) > 0.5)
            drivetrain.setLeftClimbExtensionVelocity(Math.copySign(0.025, target - leftCurrPose));
        else
            drivetrain.setLeftClimbExtensionVelocity(0.0);

        if (Math.abs(rightCurrPose - target) > 5.0)
            drivetrain.setRightClimbExtensionVelocity(Math.copySign(0.12, target - rightCurrPose));
        else if (Math.abs(rightCurrPose - target) > 0.5)
            drivetrain.setRightClimbExtensionVelocity(Math.copySign(0.025, target - rightCurrPose));
        else
            drivetrain.setRightClimbExtensionVelocity(0.0);
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