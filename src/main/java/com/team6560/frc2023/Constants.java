// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int GYRO_ID = 13;

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57785;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 8;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 4;
  // public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
  // -Math.toRadians(163.828 + 90.0 + 45.0 + 45.0 + 180.0 - 90.0);
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(254.707 + 90.0);
  // public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
  // -Math.toRadians(0.0);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
  // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
  // -Math.toRadians(77.168 + 90.0 + 45.0 - 45.0 + 180.0);
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(166.566 + 90.0);
  // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
  // -Math.toRadians(0.0);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
  // public static final double BACK_LEFT_MODULE_STEER_OFFSET =
  // -Math.toRadians(316.758 + 45.0 + 45.0 + 90.0);
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(226.758 - 180.0 + 90.0);
  // public static final double BACK_LEFT_MODULE_STEER_OFFSET =
  // -Math.toRadians(0.0);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2;
  // public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
  // -Math.toRadians(302.959 + 90.0 + 45.0 - 45.0 + 90.0 - 90.0 - 180.0);
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(63.809 + 90.0);
  // public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
  // -Math.toRadians(0.0);

  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight
   * line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final double MAX_ACCELERATION = 5.0; // m/s^2
  public static final double MAX_ANGULAR_ACCELERATION = 20.0; // rad/s^2

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  public static final int BREAK_ID = 25; // ARC motor
  public static final int CLAW_MOTOR_LEFT_ID = 26;
  public static final int CLAW_MOTOR_RIGHT_ID = 27;
  public static final int EXTENTION_SOLENOID_ID = 0;

  // public static final double ROTOR_TO_ARM = 38.1;

  public static final double BREAK_TO_ARM = 350;
  public static final double BREAK_MOTOR_MULTIPLIER = 1.0;

  public static final class VisionConstants {
    public static final double LIMELIGHT_TO_ROBOT_X = -0.1966595;
    public static final int LIMELIGHT_TO_FLOOR_DISTANCE_METERS = 0;
  }

  public static final class ControllerIds {
    public static final int FIRST_DRIVER_CONTROLLER = 0;
    public static final int SECOND_DRIVER_CONTROL_STATION = 1;
    public static final int SECOND_DRIVER_CONTROLLER = 2;

    public static final int XBOX_L_JOY_X = 0;
    public static final int XBOX_L_JOY_Y = 1;

    public static final int XBOX_R_JOY_X = 4;
    public static final int XBOX_R_JOY_Y = 5;

    public static final int XBOX_L_BUMPER = 5;
    public static final int XBOX_R_BUMPER = 6;

    public static final int XBOX_L_TRIGGER = 2;
    public static final int XBOX_R_TRIGGER = 3;

    public static final int XBOX_Y_BUTTON = 4;
    public static final int XBOX_X_BUTTON = 3;
    public static final int XBOX_B_BUTTON = 2;
    public static final int XBOX_A_BUTTON = 1;
  }

  // public static class VisionConstants {

  // // Cam mounted facing forward, half a meter forward of center, half a meter
  // up from center.
  // public static final Transform3d robotToCam = new Transform3d(
  // new Translation3d(0.5, 0.0, 0.28),
  // new Rotation3d(0, 0, 0));

  // }

  public static class FieldConstants {
    public static final double length = Units.feetToMeters(54);
    public static final double width = Units.feetToMeters(27);

    public static final double TOP_CONE_MARKER_TO_EDGE_Z_METERS = 0.98425;
    public static final double TOP_CONE_MARKER_TO_FLOOR_DISTANCE_METERS = 1.0795;
  }

}
