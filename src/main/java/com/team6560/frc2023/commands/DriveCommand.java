package com.team6560.frc2023.commands;

import java.util.ArrayList;
import java.util.Optional;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.commands.auto.AutoBuilder;
import com.team6560.frc2023.subsystems.Drivetrain;
import com.team6560.frc2023.subsystems.Limelight;
import com.team6560.frc2023.utility.Util;
import com.team6560.frc2023.utility.NetworkTable.NtValueDisplay;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

        boolean autoAlignLeft();
        boolean autoAlignRight();

        boolean driveResetGlobalPose();

        boolean overrideMaxVisionPoseCorrection();

        boolean driveIsClimbing();

        double climbVelocityL();

        double climbVelocityR();

        double driveBoostMultiplier();

        boolean driveIsAutoRotating();

        boolean isCubeMode();

        int desiredConeLocation();
    }

    private Controls controls;

    private AutoBuilder autoBuilder;

    private boolean goingToPose = false;
    private Command goToPoseAutoCommand;

    private Limelight limelight;

    private PIDController driveRotationPIDController = new PIDController(0.06, 0.05, 0.0);
    private PIDController driveTranslationYPIDController = new PIDController(0.13, 0.02, 0.0);
    private PIDController driveTranslationXPIDController = new PIDController(0.13, 0.02, 0.0);

    private boolean rotationIsPosition = true;

    private double lastRotationTheta;

    private boolean isOverridingAngle = true;

    private double lastTranslationY;

    private boolean isOverridingTranslation;

    private double lastTranslationX;

    private boolean autoAlignReady;


    private static final ArrayList<Pose2d> cubePoses = new ArrayList<>();
    private static final ArrayList<Pose2d> leftConePoses = new ArrayList<>();
    private static final ArrayList<Pose2d> rightConePoses = new ArrayList<>();

    static {
        cubePoses.add(new Pose2d(1.3751 + Units.inchesToMeters(17.75 + 3.0), 1.0715, Rotation2d.fromRotations(0.5)));
        cubePoses.add(new Pose2d(1.3751 + Units.inchesToMeters(17.75 + 3.0), 4.4243, Rotation2d.fromRotations(0.5)));
        cubePoses.add(new Pose2d(1.3751 + Units.inchesToMeters(17.75 + 3.0), 2.7479, Rotation2d.fromRotations(0.5)));
        cubePoses.add(new Pose2d(15.1603 - Units.inchesToMeters(17.75 + 3.0), 1.0715, Rotation2d.fromRotations(0.0)));
        cubePoses.add(new Pose2d(15.1603 - Units.inchesToMeters(17.75 + 3.0), 4.4243, Rotation2d.fromRotations(0.0)));
        cubePoses.add(new Pose2d(15.1603 - Units.inchesToMeters(17.75 + 3.0), 2.7479, Rotation2d.fromRotations(0.0)));


        leftConePoses.add(new Pose2d(1.3751 + Units.inchesToMeters(17.75 + 3.0), 1.6303, Rotation2d.fromRotations(0.5))); // L
        leftConePoses.add(new Pose2d(1.3751 + Units.inchesToMeters(17.75 + 3.0), 3.3091, Rotation2d.fromRotations(0.5))); // L
        leftConePoses.add(new Pose2d(1.3751 + Units.inchesToMeters(17.75 + 3.0), 4.9847, Rotation2d.fromRotations(0.5))); // L
        leftConePoses.add(new Pose2d(15.1603 - Units.inchesToMeters(17.75 + 3.0), 0.5127, Rotation2d.fromRotations(0.0))); // L
        leftConePoses.add(new Pose2d(15.1603 - Units.inchesToMeters(17.75 + 3.0), 2.1891, Rotation2d.fromRotations(0.0))); // L
        leftConePoses.add(new Pose2d(15.1603 - Units.inchesToMeters(17.75 + 3.0), 3.8655, Rotation2d.fromRotations(0.0))); // L
    
    
        rightConePoses.add(new Pose2d(1.3751 + Units.inchesToMeters(17.75 + 3.0), 0.5127, Rotation2d.fromRotations(0.5))); // R
        rightConePoses.add(new Pose2d(1.3751 + Units.inchesToMeters(17.75 + 3.0), 2.1891, Rotation2d.fromRotations(0.5))); // R
        rightConePoses.add(new Pose2d(1.3751 + Units.inchesToMeters(17.75 + 3.0), 3.8655, Rotation2d.fromRotations(0.5))); // R
        rightConePoses.add(new Pose2d(15.1603 - Units.inchesToMeters(17.75 + 3.0), 1.6303, Rotation2d.fromRotations(0.0))); // R
        rightConePoses.add(new Pose2d(15.1603 - Units.inchesToMeters(17.75 + 3.0), 3.3091, Rotation2d.fromRotations(0.0))); // R
        rightConePoses.add(new Pose2d(15.1603 - Units.inchesToMeters(17.75 + 3.0), 4.9847, Rotation2d.fromRotations(0.0))); // R
    }



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

        driveTranslationYPIDController.setIntegratorRange(-0.6, 0.6);
        driveTranslationYPIDController.setTolerance(0.0);
        driveTranslationYPIDController.enableContinuousInput(-180.0, 180.0);

        driveTranslationXPIDController.setIntegratorRange(-0.6, 0.6);
        driveTranslationXPIDController.setTolerance(0.0);
        driveTranslationXPIDController.enableContinuousInput(-180.0, 180.0);

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

        if (isOverridingAngle)
            drivetrain.driveNoX(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            controls.driveX() * controls.driveBoostMultiplier(),
                            controls.driveY() * controls.driveBoostMultiplier(),
                            calculated,
                            drivetrain.getGyroscopeRotation()));
    }

    public void translateY(double translationY) {

        lastTranslationY = translationY;

        // double thing = MathUtil.inputModulus(current - lastRotationTheta, -180, 180);

        double calculated = driveTranslationYPIDController.calculate(translationY, 0);

        if (driveTranslationYPIDController.atSetpoint()) {
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

    public void translateAndRotate(double translationY, double translationX, double rotation) {
        if (!limelight.hasTarget())
            return;
        double currentRotation = drivetrain.getGyroscopeRotation().getDegrees();

        lastRotationTheta = rotation;

        // double thing = MathUtil.inputModulus(current - lastRotationTheta, -180, 180);

        double calculatedRotation = driveRotationPIDController.calculate(currentRotation, lastRotationTheta);

        if (limelight.getTargetArea() < 0.5)
            translationX += translationX > 0 ? 14.35 : -14.35;

        lastTranslationY = translationY;
        lastTranslationX = translationX;

        // double thing = MathUtil.inputModulus(current - lastRotationTheta, -180, 180);

        double calculatedTranslationY = driveTranslationYPIDController.calculate(translationY, 0);
        double calculatedTranslationX = driveTranslationXPIDController.calculate(translationX, 0);

        // System.out.println(driveRotationPIDController.getPositionError());

        if (!limelight.hasTarget() || Math.abs(driveRotationPIDController.getPositionError()) > 7.5) {
            calculatedTranslationX = 0.0;
            calculatedTranslationY = 0.0;
        }

        drivetrain.driveNoX(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        controls.driveX(),
                        calculatedTranslationY,
                        isOverridingAngle ? calculatedRotation : controls.driveRotationX(),
                        drivetrain.getGyroscopeRotation()));
    }

    public void autoAlign() {
        double xAngle;
        double rotation;

        if (drivetrain.getPose().getX() < Constants.FieldConstants.length / 2.0) {
            xAngle = -limelight.getHorizontalAngle();
            rotation = 180.0;
        } else {
            xAngle = limelight.getHorizontalAngle();
            rotation = 0.0;
        }

        double yAngle = limelight.getVerticalAngle();

        translateAndRotate(xAngle, yAngle, rotation);
    }

    public void autoAlign2(boolean isLeft) {
        Pose2d estimatedGlobalPose = drivetrain.getPose();

        

        ArrayList<Pose2d> possibleLocations;

        if (controls.isCubeMode()) {
            possibleLocations = cubePoses;
        } else {
            if (isLeft) {
                possibleLocations = leftConePoses;
            } else {
                possibleLocations = rightConePoses;
            }
        }
        


        Pose2d desiredPose = estimatedGlobalPose.nearest(possibleLocations);

        goToPoseAutoCommand = autoBuilder.goToPose(desiredPose, new Rotation2d(0));

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (controls.autoAlignLeft() || controls.autoAlignRight()) {
            if (goToPoseAutoCommand == null) {
                autoAlign2(controls.autoAlignLeft());
                return;
            }

            if (!goingToPose) {
                drivetrain.setAutoLock(true);
                goToPoseAutoCommand.initialize();
                // System.out.println("TEST!!!");
                goingToPose = true;
            } else if (autoAlignReady)
                goToPoseAutoCommand.execute();

            if (goToPoseAutoCommand.isFinished()) {
                this.autoAlignReady = false;
                goToPoseAutoCommand = null;
                goingToPose = false;
                drivetrain.setAutoLock(false);
                drivetrain.stopModules();
            }

            return;
        } 
        else {
            this.autoAlignReady = true;
            drivetrain.setAutoLock(false);
            goToPoseAutoCommand = null;
            goingToPose = false;
        }

        if (controls.driveIsAutoRotating()) {
            double gyroRots = drivetrain.getGyroscopeRotation().getDegrees();
            if (Math.abs(Util.getHeadingDiff(gyroRots, 180.0)) < Math.abs(Util.getHeadingDiff(gyroRots, 0.0)))
                rotate(180.0);
            else
                rotate(0.0);
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

        // double poseX = drivetrain.getPose().getX();
        // Pose2d estPose = drivetrain.getLimelightEstimatedPosition();
        // Double poseXEstimated = estPose == null ? null : estPose.getX();
        // Pose2d odometryEstimatedPose = drivetrain.getOdometryPose2dNoApriltags();
        // Double odometryEstX = odometryEstimatedPose == null ? null : odometryEstimatedPose.getX();

        // if ((poseX > 0 && poseX < Constants.FieldConstants.length * 0.25) || (poseX > Constants.FieldConstants.length * 0.75 && poseX < Constants.FieldConstants.length)) {
        //     drivetrain.setOverrideMaxVisionPoseCorrection(true);

        // } else if (poseXEstimated != null && ((poseXEstimated > 0 && poseXEstimated < Constants.FieldConstants.length * 0.25) || (poseXEstimated > Constants.FieldConstants.length * 0.75 && poseXEstimated < Constants.FieldConstants.length))) {
        //     drivetrain.setOverrideMaxVisionPoseCorrection(true);
        // } else if (odometryEstX != null && ((odometryEstX > 0 && odometryEstX < Constants.FieldConstants.length * 0.25) || (odometryEstX > Constants.FieldConstants.length * 0.75 && odometryEstX < Constants.FieldConstants.length)))
        //     drivetrain.setOverrideMaxVisionPoseCorrection(true);
        // else {
        //     drivetrain.setOverrideMaxVisionPoseCorrection(controls.overrideMaxVisionPoseCorrection());
        // }

        drivetrain.setOverrideMaxVisionPoseCorrection(true);

        setClimbExtension(controls.driveIsClimbing());

        drivetrain.setBatteryBullshit(controls.driveIsClimbing());
        // drivetrain.setBatteryBullshit(false);

        if (controls.driveIsClimbing()) {

            drivetrain.setChassisState(
                    Constants.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, -controls.driveX(), 0.0)));

            // drivetrain.setLeftClimbExtensionVelocity(controls.climbVelocityL());

            // drivetrain.setRightClimbExtensionVelocity(controls.climbVelocityR());

            // drivetrain wheel radius 2in
            // climb wheel radius 1.75in

            double speed = Math.copySign(drivetrain.getAverageModuleDriveAngularTangentialSpeed(), -controls.driveX()) / Units.inchesToMeters(0.7);

            drivetrain.setClimbDriveMotorVelocity(Units.radiansPerSecondToRotationsPerMinute(speed));

        } else {
            drivetrain.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            controls.driveX() * controls.driveBoostMultiplier(),
                            controls.driveY() * controls.driveBoostMultiplier(),
                            controls.driveRotationX()
                                    * (controls.driveBoostMultiplier() > 1.0 ? 1.0 : controls.driveBoostMultiplier()),
                            drivetrain.getGyroscopeRotationNoApriltags())); // perhaps use getRawGyroRotation() instead?
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