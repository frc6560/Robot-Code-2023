// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import static com.team6560.frc2023.Constants.*;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.team6560.frc2023.Constants;
import com.team6560.frc2023.utility.NetworkTable.NtValueDisplay;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;

        private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(GYRO_ID);

        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        /**
         * The default states for each module, corresponding to an X shape.
         */
        public static final SwerveModuleState[] DEFAULT_MODULE_STATES = new SwerveModuleState[] {
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0))
        };

        /**
         * The array of swerve modules that make up this drivetrain.
         */
        public SwerveModule[] modules;

        private final Supplier<Pair<Pose2d, Double>> poseSupplier;

        private final SwerveDrivePoseEstimator poseEstimator;

        private Pose2d lastPose = new Pose2d();

        private final Field2d field = new Field2d();

        private boolean overrideMaxVisionPoseCorrection;

        private final CANSparkMax climbExtensionMotorLeft;

        private final CANSparkMax climbExtensionMotorRight;

        private final Solenoid batteryBullshit;

        private final CANSparkMax climbDriveMotorLeft;

        private CANSparkMax climbDriveMotorRight;

        private CANSparkMax[] climbDriveMotors;

        private boolean autoLock;

        private ChassisSpeeds currentManualSetChassisSpeeds;

        public Drivetrain(Supplier<Pair<Pose2d, Double>> poseSupplier) {
                this.poseSupplier = poseSupplier;

                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                NtValueDisplay.ntDispTab("Drivetrain")
                                .add("GyroscopeRotation", () -> this.getGyroscopeRotation().getDegrees())
                                .add("RawGyroRotation", () -> this.getRawGyroRotation().getDegrees());

                m_frontLeftModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
                                .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                                .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
                                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
                                .build();

                m_frontRightModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
                                .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                                .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
                                .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
                                .build();

                m_backLeftModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
                                .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                                .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
                                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                                .build();

                m_backRightModule = new MkSwerveModuleBuilder(MkModuleConfiguration.getDefaultSteerNEO())
                                .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                                .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR)
                                .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
                                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
                                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                                .build();

                modules = new SwerveModule[] { m_frontLeftModule, m_frontRightModule, m_backLeftModule,
                                m_backRightModule };

                poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
                                getRawGyroRotation(), getModulePositions(), new Pose2d(),
                                new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1), // State measurement
                                                                                                // standard deviations.
                                                                                                // X, Y, theta.
                                new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(1.75, 1.75, 1.75)); // Vision
                                                                                                    // measurement
                                                                                                    // standard
                                                                                                    // deviations.
                                                                                                    // X, Y, theta.

                climbExtensionMotorLeft = new CANSparkMax(24, CANSparkMaxLowLevel.MotorType.kBrushless);
                climbExtensionMotorLeft.setInverted(true);

                climbExtensionMotorRight = new CANSparkMax(22, CANSparkMaxLowLevel.MotorType.kBrushless);
                climbExtensionMotorRight.setInverted(true);

                climbDriveMotorLeft = new CANSparkMax(23, CANSparkMaxLowLevel.MotorType.kBrushless);
                climbDriveMotorRight = new CANSparkMax(21, CANSparkMaxLowLevel.MotorType.kBrushless);

                climbDriveMotors = new CANSparkMax[] {climbDriveMotorLeft, climbDriveMotorRight};
                // for (CANSparkMax i : new CANSparkMax[] {climbExtensionMotorLeft,
                //                 climbExtensionMotorRight }) {
                //         i.getPIDController().setP(0.005);
                //         i.getPIDController().setI(0.00005);
                //         i.getPIDController().setD(0.0);

                //         i.getPIDController().setIZone(0.0);
                // }

                
                for (CANSparkMax i : climbDriveMotors) {
                        i.restoreFactoryDefaults();
                        i.getPIDController().setFF(0.001);
                        i.getPIDController().setP(0.0);
                        i.getPIDController().setI(0.0);
                        i.getPIDController().setD(0.0);

                        i.getPIDController().setIZone(0.0);
                }
                
                climbDriveMotorLeft.setInverted(false);
                climbDriveMotorRight.setInverted(true);

                batteryBullshit = new Solenoid(PneumaticsModuleType.CTREPCM, 1); //TODO: CHANGE

                resetOdometry(new Pose2d());

                SmartDashboard.putData("Field", field);

                NtValueDisplay.ntDispTab("Climb")
                                .add("isClimbExtended", () -> isClimbExtended())
                                .add("leftClimbPosition", () -> getLeftClimbPosition())
                                .add("rightClimbPosition", () -> getRightClimbPosition())
                                .add("climbMotorVelocityRPM", () -> getClimbDriveMotorVelocityRPM())
                                .add("batteryBullshitExtended", () -> isBatteryBullshitExtended());
        }

        public boolean isClimbExtended() {
                return Math.abs(climbExtensionMotorLeft.getEncoder().getPosition()) > 0.0;
        }

        public double getLeftClimbPosition() {
                return climbExtensionMotorLeft.getEncoder().getPosition();
        }

        public double getRightClimbPosition() {
                return climbExtensionMotorRight.getEncoder().getPosition();
        }

        public void setLeftClimbExtensionVelocity(double velocityPercentOutput) {
                climbExtensionMotorLeft.set(velocityPercentOutput);
        }

        public void setRightClimbExtensionVelocity(double velocityPercentOutput) {
                climbExtensionMotorRight.set(velocityPercentOutput);
        }

        public void setClimbDriveMotorVelocity(double velocityRPM) {
                for (CANSparkMax i : climbDriveMotors)
                        i.getPIDController().setReference(velocityRPM, ControlType.kVelocity);
        }

        public double getClimbDriveMotorVelocityRPM() {
                return climbDriveMotorLeft.getEncoder().getVelocity();
        }

        public void setBatteryBullshit(boolean isClimbing) {
                batteryBullshit.set(isClimbing);
        }

        public boolean isBatteryBullshitExtended() {
                return batteryBullshit.get();
        }

        public double getAverageModuleDriveAngularTangentialSpeed() {
                double sum = 0;
                double sign = 0;
                for (SwerveModule i : modules) {
                        sign += i.getDriveVelocity();
                        sum += Math.abs(i.getDriveVelocity());
                }
                return Math.copySign(sum / modules.length, sign);
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                // m_navx.zeroYaw();
                // if (poseEstimator == null)
                // m_navx.zeroYaw();
                resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d(0.0)));
        }

        public Rotation2d getRawGyroRotation() {
                return Rotation2d.fromDegrees(pigeon.getYaw());
        }

        /**
         * Gets the current rotation of the robot according to the gyroscope.
         *
         * @return The rotation (yaw) of the robot
         **/
        public Rotation2d getGyroscopeRotation() {
                // if (m_navx.isMagnetometerCalibrated()) {
                // // We will only get valid fused headings if the magnetometer is calibrated
                // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                // }
                // We will only get valid fused headings if the magnetometer is calibrated
                // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.

                // if pose estimator is null, default to the raw gyro rotation
                if (poseEstimator == null) {
                        if (lastPose == null) {
                                return new Rotation2d();
                        }
                        return lastPose.getRotation();
                }

                return poseEstimator.getEstimatedPosition().getRotation();
                // return getRawGyroRotation();
        }

        public SwerveModulePosition[] getModulePositions() {
                // return reverseModulePositionArray(new SwerveModulePosition[] {
                // m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                // m_backLeftModule.getPosition(), m_backRightModule.getPosition() });

                return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                                m_backLeftModule.getPosition(), m_backRightModule.getPosition() };

        }

        /**
         * Sets drive motor idle mode to be either brake mode or coast mode.
         * 
         * @param brake If true, sets brake mode, otherwise sets coast mode
         */
        public void setDriveMotorBrakeMode(boolean brake) {
                IdleMode sparkMaxMode = brake ? IdleMode.kBrake : IdleMode.kCoast;
                NeutralMode phoenixMode = brake ? NeutralMode.Brake : NeutralMode.Coast;

                for (SwerveModule i : modules) {
                        if (i.getSteerMotor() instanceof CANSparkMax)
                                ((CANSparkMax) i.getSteerMotor()).setIdleMode(IdleMode.kCoast);
                        else
                                ((TalonFX) i.getSteerMotor()).setNeutralMode(NeutralMode.Coast);

                        if (i.getDriveMotor() instanceof CANSparkMax)
                                ((CANSparkMax) i.getDriveMotor()).setIdleMode(sparkMaxMode);
                        else
                                ((TalonFX) i.getDriveMotor()).setNeutralMode(phoenixMode);

                }

        }

        public Rotation2d getPitch() {
                return Rotation2d.fromDegrees(pigeon.getPitch());
        }

        public Rotation2d getRoll() {
                return Rotation2d.fromDegrees(pigeon.getRoll());
        }

        /**
         * ONLY USE IN DEBUGGING!
         * Used to reverse the distance read in a SwerveModulePosition array
         * 
         * @param array the SwerveModulePosition[] array you would like thre reversed
         *              values of
         * @return the reversed SwerveModulePosition[] array
         */
        @Deprecated
        public SwerveModulePosition[] reverseModulePositionArray(SwerveModulePosition[] array) {
                SwerveModulePosition[] output = new SwerveModulePosition[array.length];

                for (int i = 0; i < array.length; i++)
                        output[i] = new SwerveModulePosition(-array[i].distanceMeters, array[i].angle);

                return output;
        }

        /**
         * 
         * This method is used to control the movement of the chassis.
         * 
         * @param chassisSpeeds an object containing the desired speeds of the chassis
         *                      in the X and Y directions, as well as the desired
         *                      rotational speed
         */
        public void drive(ChassisSpeeds chassisSpeeds) {
                if (driveNoX(chassisSpeeds))
                        setChassisState(DEFAULT_MODULE_STATES);
        }

        public boolean driveNoX(ChassisSpeeds chassisSpeeds) {
                this.currentManualSetChassisSpeeds = chassisSpeeds;
                if (this.autoLock)
                        return false;
                SwerveModuleState[] states = Constants.m_kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                if (DriverStation.isAutonomous()) {
                        setChassisState(states);
                        return false;
                }

                // chassisSpeeds = new
                // ChassisSpeeds(xLimiter.calculate(chassisSpeeds.vxMetersPerSecond),
                // yLimiter.calculate(chassisSpeeds.vyMetersPerSecond),
                // rotLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond));

                for (SwerveModuleState state : states) {
                        if (state.speedMetersPerSecond > 0.05) {
                                setChassisState(states);
                                return false;
                        }
                }

                return true;
        }

        @Override
        public void periodic() {
                updateOdometry();

                lastPose = poseEstimator.getEstimatedPosition();

                field.setRobotPose(getPose());
        }

        /**
         * Sets the speeds and orientations of each swerve module.
         * array order: front left, front right, back left, back right
         *
         * @param moduleStates The desired states for each module.
         */
        public void setChassisState(SwerveModuleState[] states) {

                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

        }

        /**
         * ONLY FOR AUTO!!:
         * Sets the speeds and orientations of each swerve module.
         * array order: front left, front right, back left, back right
         *
         * This is likely required because of a bug in pathplannerlib, where angular
         * speed is negative.
         * 
         * But hey, if it aint broke dont fix it?
         * 
         * @param moduleStates The desired states for each module.
         */
        public void autoSetChassisState(SwerveModuleState[] states) {
                setChassisState(states);
                // ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(states);
                // setChassisState(m_kinematics.toSwerveModuleStates(new
                // ChassisSpeeds(speeds.vxMetersPerSecond,
                // speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond)));
        }

        public void setChassisState(double fLdeg, double fRdeg, double bLdeg, double bRdeg) {
                setChassisState(
                                new SwerveModuleState[] {
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(fLdeg)),
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(fRdeg)),
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(bLdeg)),
                                                new SwerveModuleState(0.0, Rotation2d.fromDegrees(bRdeg))
                                });
        }

        /**
         * Gets the current pose of the robot according to the pose estimator
         * calculations.
         *
         * @return The pose of the robot.
         */
        public Pose2d getPose() {
                // System.out.println(poseEstimator.getEstimatedPosition());
                return poseEstimator.getEstimatedPosition();
        }

        public ChassisSpeeds getChassisSpeeds() {
                return Constants.m_kinematics.toChassisSpeeds(getStates());
        }

        public SwerveModuleState[] getStates() {
                return new SwerveModuleState[] { m_frontLeftModule.getState(), m_frontRightModule.getState(),
                                m_backLeftModule.getState(), m_backRightModule.getState() };
        }

        /**
         * 
         * This method is used to reset the position of the robot's pose estimator.
         * 
         * @param pose the new pose to use as the starting position for the pose
         *             estimator
         */
        public void resetOdometry(Pose2d pose) {
                poseEstimator.resetPosition(
                                getRawGyroRotation(),
                                getModulePositions(), pose);
        }

        /**
         * 
         * This method is used to stop all of the swerve drive modules.
         */
        public void stopModules() {
                for (SwerveModule i : modules) {
                        i.set(0.0, i.getSteerAngle());
                }
        }

        public void setOverrideMaxVisionPoseCorrection(boolean overided) {
                this.overrideMaxVisionPoseCorrection = overided;
        }

        /** Updates the field-relative position. */
        private void updateOdometry() {
                poseEstimator.update(getRawGyroRotation(), getModulePositions());

                // Also apply vision measurements. We use 0.3 seconds in the past as an example
                // -- on
                // a real robot, this must be calculated based either on latency or timestamps.
                Pair<Pose2d, Double> result = poseSupplier.get();
                if (result == null)
                        return;

                Pose2d camPose = result.getFirst();
                
                if (camPose == null || camPose == new Pose2d())
                        return;
                
                if (camPose.minus(getPose()).getTranslation().getNorm() > 1.5 && !overrideMaxVisionPoseCorrection)
                        return;

                // if (!overrideMaxVisionPoseCorrection) {
                //         // TODO: test this line of code and see if it's stupid or not
                //         camPose = new Pose2d(camPose.getTranslation(), getGyroscopeRotation());
                // }

                double camPoseObsTime = result.getSecond();
                poseEstimator.addVisionMeasurement(camPose, camPoseObsTime);
        }

        public void teleopFinesseChassisState(SwerveModuleState[] state) {
                ChassisSpeeds speeds = Constants.m_kinematics.toChassisSpeeds(state);

                setChassisState(Constants.m_kinematics.toSwerveModuleStates(new ChassisSpeeds(currentManualSetChassisSpeeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond)));
        }

        public void setAutoLock(boolean lock) {
                this.autoLock = lock;
        }
}