// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import static com.team6560.frc2023.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.team6560.frc2023.Constants;
import com.team6560.frc2023.utility.NetworkTable.NtValueDisplay;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
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

        /**
         * The AHRS object used to get the current orientation of the robot.
         */
        private final AHRS m_navx = new AHRS(Port.kMXP, (byte) 200);

        // These are our swerve modules. We initialize them in the constructor.
        private SwerveModule m_frontLeftModule;
        private SwerveModule m_frontRightModule;
        private SwerveModule m_backLeftModule;
        private SwerveModule m_backRightModule;

        /**
         * The default states for each module, corresponding to an X shape.
         */
        public static final SwerveModuleState[] DEFAULT_MODULE_STATES = new SwerveModuleState[] {
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
        };

        /**
         * The array of swerve modules that make up this drivetrain.
         */
        public SwerveModule[] modules;

        private PhotonCameraWrapper pcw = new PhotonCameraWrapper();

        private SwerveDrivePoseEstimator poseEstimator;
        /**
         * The offset to apply to the gyroscope readings to account for any drift.
         */
        private static final double GYRO_OFFSET = 3600.0 / (3473.0);


        private final Field2d field = new Field2d();

        private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
        private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
        private SlewRateLimiter rotLimiter = new SlewRateLimiter(6.0);

        /**
         * Constructs a new `Drivetrain` object and initializes the swerve modules.
         */
        public Drivetrain() {

                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                NtValueDisplay.ntDispTab("Drivetrain")
                                .add("Yaw Function", () -> this.getGyroscopeRotation().getDegrees())
                                .add("Raw Yaw", () -> m_navx.getYaw())
                                .add("Continuous Yaw", () -> m_navx.getRotation2d().getDegrees() * GYRO_OFFSET);

                m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                FRONT_LEFT_MODULE_STEER_MOTOR,
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);

                modules = new SwerveModule[] { m_frontLeftModule, m_frontRightModule, m_backLeftModule,
                                m_backRightModule };

                // TODO: Update standard deviation so it's less jiggly
                // TODO: Also investigate different AprilTag methods in PhotonCameraWrapper
                poseEstimator = new SwerveDrivePoseEstimator(Constants.m_kinematics,
                        getGyroscopeRotation(), getModulePositions(), new Pose2d());

                SmartDashboard.putData("Field", field);
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                m_navx.zeroYaw();
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
                return new Rotation2d(m_navx.getYaw() * -1 / 180 * Math.PI);
        }

        public SwerveModulePosition[] getModulePositions() {
                return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                                m_backLeftModule.getPosition(), m_backRightModule.getPosition() };
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
                
                SwerveModuleState[] states = Constants.m_kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                if (DriverStation.isAutonomous()) {
                        setChassisState(states);
                        return;
                }

                chassisSpeeds = new ChassisSpeeds(xLimiter.calculate(chassisSpeeds.vxMetersPerSecond),
                                yLimiter.calculate(chassisSpeeds.vyMetersPerSecond), rotLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond));


                for (SwerveModuleState state : states) {
                        if (state.speedMetersPerSecond > 0.05) {
                                setChassisState(states);
                                return;
                        }
                }

                setChassisState(DEFAULT_MODULE_STATES);

        }

        @Override
        public void periodic() {
                updateOdometry();

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
         * Gets the current pose of the robot according to the pose estimator calculations.
         *
         * @return The pose of the robot.
         */
        public Pose2d getPose() {
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
         * @param pose the new pose to use as the starting position for the pose estimator
         */
        public void resetOdometry(Pose2d pose) {
                poseEstimator.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
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

        /** Updates the field-relative position. */
        private void updateOdometry() {
                poseEstimator.update(getGyroscopeRotation(), getModulePositions());

                // Also apply vision measurements. We use 0.3 seconds in the past as an example
                // -- on
                // a real robot, this must be calculated based either on latency or timestamps.
                Pair<Pose2d, Double> result = pcw.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
                Pose2d camPose = result.getFirst();
                var camPoseObsTime = result.getSecond();
                if (camPose != null) {
                        poseEstimator.addVisionMeasurement(camPose, camPoseObsTime);
                }

        }
}