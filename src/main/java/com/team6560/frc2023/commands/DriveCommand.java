package com.team6560.frc2023.commands;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.commands.auto.AutoBuilder;
import com.team6560.frc2023.subsystems.Drivetrain;
import com.team6560.frc2023.subsystems.Limelight;
import com.team6560.frc2023.utility.NetworkTable.NtValueDisplay;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

        double driveRotationX();

        double driveRotationY();

        boolean driveResetYaw();

        boolean autoAlign();

        boolean driveResetGlobalPose();

        boolean overrideMaxVisionPoseCorrection();

        boolean driveIsClimbing();

        double climbVelocityL();

        double climbVelocityR();

        double driveBoostMultiplier();
    }

    private Controls controls;

    private AutoBuilder autoBuilder;

    private boolean goingToPose = false;
    private Command goToPoseAutoCommand;

    private Limelight limelight;

    private PIDController driveRotationPIDController = new PIDController(0.03, 0.0269420, 0.0);
    private PIDController driveTranslationPIDController = new PIDController(0.1, 0.0, 0.00025);

    private boolean rotationIsPosition = true;

    private double lastRotationTheta;

    private boolean isOverridingAngle = true;

    private double lastTranslationX;

    private boolean isOverridingTranslation;

    /**
     * Creates a new `DriveCommand` instance.
     *
     * @param drivetrainSubsystem the `Drivetrain` subsystem used by the command
     * @param controls            the controls for the command
     */
    public DriveCommand(Drivetrain drivetrainSubsystem, AutoBuilder autoBuilder, Limelight limelight,
            Controls controls) {
        this.drivetrain = drivetrainSubsystem;
        this.autoBuilder = autoBuilder;
        this.controls = controls;
        this.limelight = limelight;

        addRequirements(drivetrainSubsystem);

        driveRotationPIDController.setIntegratorRange(-0.5, 0.5);
        // driveRotationPIDController.setTolerance(3.5);
        driveRotationPIDController.setTolerance(0.0);
        driveRotationPIDController.enableContinuousInput(-180.0, 180.0);

        driveTranslationPIDController.setIntegratorRange(-0.6, 0.6);
        driveTranslationPIDController.setTolerance(0.0);
        driveTranslationPIDController.enableContinuousInput(-180.0, 180.0);

        NtValueDisplay.ntDispTab("DriveCommand")
                .add("actual", () -> drivetrainSubsystem.getGyroscopeRotation().getDegrees())
                .add("desired", () -> {
                    return this.lastRotationTheta;
                });
    }

    public void rotate(double rotation) {
        double current = drivetrain.getGyroscopeRotation().getDegrees();

        lastRotationTheta = rotation;

        // double thing = MathUtil.inputModulus(current - lastRotationTheta, -180, 180);

        double calculated = driveRotationPIDController.calculate(current, lastRotationTheta);

        if (driveRotationPIDController.atSetpoint()) {
            isOverridingAngle = false;
        } else
            isOverridingAngle = true;

        if (isOverridingAngle)
            drivetrain.driveNoX(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            controls.driveX(),
                            controls.driveY(),
                            calculated,
                            drivetrain.getGyroscopeRotation()));
    }

    public void translateX(double translationX) {

        lastTranslationX = translationX;

        // double thing = MathUtil.inputModulus(current - lastRotationTheta, -180, 180);

        double calculated = driveTranslationPIDController.calculate(translationX, 0);

        if (driveTranslationPIDController.atSetpoint()) {
            isOverridingTranslation = false;
        } else
            isOverridingTranslation = true;

        if (isOverridingTranslation)
            drivetrain.driveNoX(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            controls.driveX(),
                            calculated,
                            controls.driveRotationX(),
                            drivetrain.getGyroscopeRotation()));
    }

    public void translateAndRotate(double translationX, double rotation) {
        double currentRotation = drivetrain.getGyroscopeRotation().getDegrees();

        lastRotationTheta = rotation;

        // double thing = MathUtil.inputModulus(current - lastRotationTheta, -180, 180);

        double calculatedRotation = driveRotationPIDController.calculate(currentRotation, lastRotationTheta);

        if (driveRotationPIDController.atSetpoint()) {
            isOverridingAngle = false;
        } else
            isOverridingAngle = true;

        lastTranslationX = translationX;

        // double thing = MathUtil.inputModulus(current - lastRotationTheta, -180, 180);

        double calculatedTranslation = driveTranslationPIDController.calculate(translationX, 0);


        // if (driveTranslationPIDController.atSetpoint()) {
        //     drivetrain.driveNoX(
        //         ChassisSpeeds.fromFieldRelativeSpeeds(
        //                 controls.driveX(),
        //                 0.0,
        //                 isOverridingAngle ? calculatedRotation : controls.driveRotationX(),
        //                 drivetrain.getGyroscopeRotation()));
        //     return;
        // } 


        drivetrain.driveNoX(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        controls.driveX(),
                        calculatedTranslation,
                        isOverridingAngle ? calculatedRotation : controls.driveRotationX(),
                        drivetrain.getGyroscopeRotation()));
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (controls.autoAlign()) {
            // if (Math.abs(controls.driveRotationX()) > 0.1 ||
            // Math.abs(controls.driveRotationY()) > 0.1)
            // rotate(Math.toDegrees(90.0 - Math.atan2(controls.driveRotationY(),
            // controls.driveRotationX())));

            double xAngle = -limelight.getHorizontalAngle();
            double rotation = 180.0;
            translateAndRotate(xAngle, rotation);
            return;
        }

        // if (controls.autoAlign() && !goingToPose) {
        // goToPoseAutoCommand = autoBuilder.goToPose(new Pose2d(new Translation2d(0,
        // limelight.getXDistMeters()), Rotation2d.fromDegrees(0.0)));
        // goToPoseAutoCommand.initialize();
        // goingToPose = true;

        // }
        // if (goingToPose && goToPoseAutoCommand != null &&
        // !goToPoseAutoCommand.isFinished()) {
        // goToPoseAutoCommand.execute();
        // return;
        // }
        // goingToPose=false;

        if (controls.driveResetYaw()) {
            drivetrain.zeroGyroscope();
        }

        if (controls.driveResetGlobalPose())
            drivetrain.resetOdometry(new Pose2d());

        drivetrain.setOverrideMaxVisionPoseCorrection(controls.overrideMaxVisionPoseCorrection());

        setClimbExtension(controls.driveIsClimbing());

        // drivetrain.setBatteryBullshit(controls.driveIsClimbing());
        drivetrain.setBatteryBullshit(false);

        if (controls.driveIsClimbing()) {

            drivetrain.setChassisState(
                    Constants.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, -controls.driveY(), 0.0)));

            // drivetrain.setLeftClimbExtensionVelocity(controls.climbVelocityL());

            // drivetrain.setRightClimbExtensionVelocity(controls.climbVelocityR());

            // drivetrain wheel radius 2in
            // climb wheel radius 1.75in

            drivetrain.setClimbDriveMotorVelocity(Units.radiansPerSecondToRotationsPerMinute(
                    drivetrain.getAverageModuleDriveAngularTangentialSpeed() / Units.inchesToMeters(0.7)));

        } else {
            drivetrain.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            controls.driveX() * controls.driveBoostMultiplier(),
                            controls.driveY() * controls.driveBoostMultiplier(),
                            controls.driveRotationX() * (controls.driveBoostMultiplier() > 1.0 ? 1.0 : controls.driveBoostMultiplier()),
                            drivetrain.getGyroscopeRotation())); // perhaps use getRawGyroRotation() instead?
        }

    }

    public void setClimbExtension(boolean extended) {
        double leftCurrPose = drivetrain.getLeftClimbPosition();
        double rightCurrPose = drivetrain.getRightClimbPosition();

        double target = extended ? 100.0 : 0.0;

        if (Math.abs(leftCurrPose - target) > 5.0)
            drivetrain.setLeftClimbExtensionVelocity(Math.copySign(0.12, target - leftCurrPose));
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