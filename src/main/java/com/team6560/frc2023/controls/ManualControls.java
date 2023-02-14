// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.controls;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.Constants.*;
import com.team6560.frc2023.subsystems.Arm.ArmPose;
import com.team6560.frc2023.subsystems.Limelight;
import com.team6560.frc2023.commands.DriveCommand;
import com.team6560.frc2023.utility.NumberStepper;
import com.team6560.frc2023.commands.ArmCommand;
import com.team6560.frc2023.utility.PovNumberStepper;
import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class ManualControls implements DriveCommand.Controls, Limelight.Controls, ArmCommand.Controls {
  private XboxController xbox;

  private final PovNumberStepper speed;
  private final PovNumberStepper turnSpeed;

  private NetworkTable limelightTable;

  private NetworkTable climbTable;

  private XboxController controlStation;

  /**
   * Creates a new `ManualControls` instance.
   *
   * @param xbox the Xbox controller to use for manual control
   */
  public ManualControls(XboxController xbox) {
    this(xbox, null);
  }

  public ManualControls(XboxController xbox, XboxController controlStation) {
    this.xbox = xbox;
    if (controlStation == null)
      this.controlStation = xbox;
    else
      this.controlStation = controlStation;

    this.speed = new PovNumberStepper(
        new NumberStepper(Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.2, 0.0,
            Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.6, Constants.MAX_VELOCITY_METERS_PER_SECOND * 0.05),
        xbox,
        PovNumberStepper.PovDirection.VERTICAL);

    this.turnSpeed = new PovNumberStepper(
        new NumberStepper(Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.1, 0.0,
            Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.15,
            Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.0025),
        xbox,
        PovNumberStepper.PovDirection.HORIZONTAL);
    
    ntDispTab("Controls")
      .add("Y Joystick", this::driveY)
      .add("X Joystick", this::driveX)
      .add("Rotation Joystick", this::driveRotation);

    
    limelightTable = NetworkTableInstance.getDefault().getTable("Limelight");

    limelightTable.getEntry("limelightPipeline").setInteger( (long) 0);
    
    climbTable = NetworkTableInstance.getDefault().getTable("Climb");

    climbTable.getEntry("isClimbing").setBoolean(false);

    climbTable.getEntry("climbVelocityL").setDouble(0.0);

    climbTable.getEntry("climbVelocityR").setDouble(0.0);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.001);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /**
   * Returns the x component of the robot's velocity, as controlled by the Xbox
   * controller.
   *
   * @return the x component of the robot's velocity
   */
  @Override
  public double driveX() {
    return modifyAxis(-xbox.getLeftY() * speed.get());
  }

  /**
   * Returns the y component of the robot's velocity, as controlled by the Xbox
   * controller.
   *
   * @return the y component of the robot's velocity
   */
  @Override
  public double driveY() {
    return modifyAxis(-xbox.getLeftX() * speed.get());
  }

  /**
   * Returns the angular velocity of the robot, as controlled by the Xbox
   * controller.
   *
   * @return the angular velocity of the robot
   */
  @Override
  public double driveRotation() {
    return modifyAxis(-xbox.getRightX() * turnSpeed.get());
  }

  /**
   * Returns whether the yaw of the robot's gyroscope should be reset, as
   * controlled by the Xbox controller.
   *
   * @return whether the yaw of the robot's gyroscope should be reset
   */
  @Override
  public boolean driveResetYaw() {
    return xbox.getStartButton();
  }

  @Override
  public boolean GoToDoubleSubstation() {
    return xbox.getAButton();
  }

  @Override
  public boolean driveResetGlobalPose() {
    return xbox.getBackButton();
  }

  @Override
  public int getLimelightPipeline() {
    return (int) limelightTable.getEntry("limelightPipeline").getInteger( (long) 0);
  }

  @Override
  public boolean overrideMaxVisionPoseCorrection() {
    return xbox.getYButton();
  }

  @Override
  public double armRotationOverride(){
    return controlStation.getLeftTriggerAxis() - controlStation.getRightTriggerAxis();
  }

  @Override
  public boolean armExtentionOverride(){
    return controlStation.getXButton();
  }

  @Override
  public double runClaw(){
    return (controlStation.getRightBumper() ? 1 : (controlStation.getLeftBumper() ? -1 : 0));
  }

  @Override
  public boolean driveIsClimbing() {
    return this.climbTable.getEntry("isClimbing").getBoolean(false);
  }

  @Override
  public double climbVelocityL() {
    return this.climbTable.getEntry("climbVelocityL").getDouble(0.0);
  }


  @Override
  public double climbVelocityR() {
    return this.climbTable.getEntry("climbVelocityR").getDouble(0.0);
  }

  @Override
  public ArmPose armState() {
    // if (!controlStation.getRawButton(4) || !controlStation.getRawButton(1))
    //   return ArmPose.ZERO;


    return xbox.getBButton() ? ArmPose.MEDIUM_CONE : ArmPose.ZERO;

    // if (controlStation.getRawButton(3))
    //   return ArmPose.ZERO;

    // if (controlStation.getRawAxis(ControllerIds.DRIVER_STATION_X_AXIS) == -1)
    //   return ArmPose.HUMAN_PLAYER;
    
    // if (controlStation.getRawButton(5)) {
    //   switch ((int) controlStation.getRawAxis(ControllerIds.DRIVER_STATION_Y_AXIS)) {
    //     case 1:
    //     return ArmPose.HIGH_CONE;
    //     case 0:
    //       return ArmPose.MEDIUM_CONE;
    //     case -1:
    //       return ArmPose.LOW;
    //     default:
    //       return ArmPose.ZERO;
    //   }
    // }

    // switch ((int) controlStation.getRawAxis(ControllerIds.DRIVER_STATION_Y_AXIS)) {
    //   case 1:
    //   return ArmPose.HIGH_CUBE;
    //   case 0:
    //     return ArmPose.MEDIUM_CUBE;
    //   case -1:
    //     return ArmPose.LOW;
    //   default:
    //     return ArmPose.ZERO;
    // }
    
    
  }

  @Override
  public boolean isOverridingArm() {
    // TODO Auto-generated method stub
    return false;
  }

}
