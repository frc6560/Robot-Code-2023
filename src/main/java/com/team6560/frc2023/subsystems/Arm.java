// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.team6560.frc2023.Constants;

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team6560.frc2023.Constants.*;

public class Arm extends SubsystemBase {
  /*
   * rotors: 38.1
   * break motor: 350
   * extention piston
   * gripper?
   */

  /*
   * rotate hand up
   * move rotors & break motor
   * extend & retract arm
   * run claw
   */

  CANSparkMax breakMotor = new CANSparkMax(BREAK_ID, MotorType.kBrushless);
  SparkMaxPIDController breakMotorPid = breakMotor.getPIDController();

  CANSparkMax clawMotorL = new CANSparkMax(CLAW_MOTOR_LEFT_ID, MotorType.kBrushless);
  CANSparkMax clawMotorR = new CANSparkMax(CLAW_MOTOR_RIGHT_ID, MotorType.kBrushless);

  Solenoid extentionPiston = new Solenoid(PneumaticsModuleType.CTREPCM, EXTENTION_SOLENOID_ID);

  NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");

  NetworkTableEntry breakMultiplyer;

  NetworkTableEntry ntTopLimit;
  NetworkTableEntry ntBottomLimit;

  NetworkTableEntry invertClaw;
  private double currentReference;

  // private PIDController armPidController = new PIDController(25.0, 7.25, 6.0);

  /** Creates a new Arm. */
  public Arm() {
    breakMotor.restoreFactoryDefaults();
    // breakMotor.setInverted(true);
    breakMotor.setIdleMode(IdleMode.kBrake);
    // breakMotor.setInverted(true);

    clawMotorL.restoreFactoryDefaults();
    clawMotorL.setIdleMode(IdleMode.kBrake);

    clawMotorR.restoreFactoryDefaults();
    clawMotorR.setIdleMode(IdleMode.kBrake);
    clawMotorR.setInverted(true);

    ntDispTab("Arm")
        .add("Break Motor Speed", this::getBreakMotorSpeed)
        .add("Claw Speed Left", this::getClawSpeedL)
        .add("Claw Speed Right", this::getClawSpeedR)
        .add("Extention Status", this::getExtentionStatus)
        .add("raw arm pos", this::getRawArmPose)
        .add("armPose", this::getArmPose)
        .add("Target Pos", () -> currentReference);

    breakMultiplyer = ntTable.getEntry("Break Motor Multiplyer");
    breakMultiplyer.setDouble(0.1);

    ntTopLimit = ntTable.getEntry("Top Soft Limit");
    // ntTopLimit.setDouble(107.0);
    ntTopLimit.setDouble(Constants.ArmConstants.DEFAULT_TOP_SOFT_LIMIT);
    ntBottomLimit = ntTable.getEntry("Bottom Soft Limit");
    ntBottomLimit.setDouble(0.0);

    invertClaw = ntTable.getEntry("Invert Claw?");
    invertClaw.setBoolean(false);


    // armPidController.disableContinuousInput();
    // armPidController.setIntegratorRange(-0.5, 0.5);
    // armPidController.setTolerance(0.05);

    breakMotor.setOpenLoopRampRate(Constants.ArmConstants.zeroToFullTime);

    breakMotorPid.setP(6.560e-8, 0);
    breakMotorPid.setI(1.06560e-9, 0);
    breakMotorPid.setD(6.560e-12, 0);
    breakMotorPid.setFF(0.001, 0);

    breakMotorPid.setSmartMotionMaxAccel(656.0, 0);
    breakMotorPid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    // breakMotorPid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    breakMotorPid.setSmartMotionMaxVelocity(1000, 0);
    // breakMotorPid.setSmartMotionMinOutputVelocity(50, 0);
    // breakMotorPid.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR, 0);

    breakMotorPid.setP(0.05, 1);
    breakMotorPid.setI(1e-6, 1);
    breakMotorPid.setD(0, 1);
    breakMotorPid.setFF(0.000156, 1);

  }

  @Override
  public void periodic() {
  }

  public void setArmExtention(boolean status) {
    // System.out.println("Arm extended " + (status ? "Out." : "In."));

    extentionPiston.set(status);
  }

  public double convertRawArmPoseToArmPose(double rawArmPose) {
    double low = ntBottomLimit.getDouble(0.0);
    double high = ntTopLimit.getDouble(Constants.ArmConstants.DEFAULT_TOP_SOFT_LIMIT);

    return (rawArmPose - low) / (high - low);
  }

  public double convertArmPoseToRawArmPose(double armPose) {
    double low = ntBottomLimit.getDouble(0.0);
    double high = ntTopLimit.getDouble(Constants.ArmConstants.DEFAULT_TOP_SOFT_LIMIT);

    return armPose * (high - low) + low;
  }

  /** Sets arm break velocity in RPM */
  public void setBreakMotor(double rpm) {
    double speed = rpm * breakMultiplyer.getDouble(1.0) * BREAK_MOTOR_MULTIPLIER;

    // if(speed != 0) System.out.println("break is running at " + speed);

    breakMotor.set(-speed);
  }

  public void setClawSpeed(double output) {
    // if(output != 0) System.out.println("Claw is running at " + output);

    clawMotorL.set(output);
    clawMotorR.set(output);
  }

  public void setArmRotationVelocity(double output) {
    if (getArmPose() <= Constants.ArmConstants.armPoseMap.get(Constants.ArmConstants.ArmPose.DEFAULT).getPosition()) {
      output = Math.min(0, output);

    } else if (getArmPose() >= 1.0) {
      output = Math.max(0, output);
    }

    setBreakMotor(output * BREAK_TO_ARM);
  }

  public void setArmRotationVelocityOverrideSoftLimits(double output) {
    setBreakMotor(output * BREAK_TO_ARM);
  }

  public void setArmState(Constants.ArmConstants.ArmPose armPose) {
    setArmState(Constants.ArmConstants.armPoseMap.get(armPose).getPosition());
  }

  public void setArmState(double pose) {
    // setArmRotation(pose);
    // if(!ntMarkRadin.getBoolean(false)) setArmRotationMark(pose); else
    // setArmRotationRadin(pose);
    setArmRotation(pose);

  }

  public void setArmRotation(Constants.ArmConstants.ArmPose armPose) {
    setArmRotation(Constants.ArmConstants.armPoseMap.get(armPose).getPosition());
  }

  public void setArmRotationOld(double pose) {
    this.currentReference = pose;
    breakMotorPid.setReference(currentReference * (ntTopLimit.getDouble(Constants.ArmConstants.DEFAULT_TOP_SOFT_LIMIT)
        - ntBottomLimit.getDouble(Constants.ArmConstants.armPoseMap.get(Constants.ArmConstants.ArmPose.ZERO).getPosition())), ControlType.kSmartMotion);
    // double calculated = -armPidController.calculate(getArmPose(), pose);
    // if (armPidController.atSetpoint())
    // return;
    // setBreakMotorVolts(calculated);
  }

  public void setArmRotation(double pose) {
    this.currentReference = pose;
    double rot = currentReference * (ntTopLimit.getDouble(Constants.ArmConstants.DEFAULT_TOP_SOFT_LIMIT)
        - ntBottomLimit.getDouble(Constants.ArmConstants.armPoseMap.get(Constants.ArmConstants.ArmPose.ZERO).getPosition()));

    final double rampUpWindow = 750;
    if (Math.abs(getBreakMotorSpeed()) < rampUpWindow && !isArmAtSetpoint()) {
      setArmRotationOld(pose); // ramp up

    } else {
      breakMotorPid.setReference(rot, ControlType.kPosition, 1);
    }

  }

  public void resetArmZero() {
    breakMotor.getEncoder().setPosition(0.0);
  }

  ProfiledPIDController controller = new ProfiledPIDController(breakMotorPid.getP(1), breakMotorPid.getI(1) * 5,
      breakMotorPid.getD(1), new TrapezoidProfile.Constraints(11000, 4000));

  public void setArmRotationMark(double pose) {
    controller.disableContinuousInput();
    double ff = breakMotorPid.getFF(1);
    double volts = (controller.calculate(breakMotor.getEncoder().getPosition(), convertArmPoseToRawArmPose(pose)) + ff);
    // System.out.println("Running marks at: " + volts);

    breakMotor.setVoltage(volts);
  }

  public boolean isArmAtSetpoint() {
    boolean isAtReference = Math.abs(currentReference - getArmPose()) < convertRawArmPoseToArmPose(Constants.ArmConstants.ALLOWED_ERROR);
    // System.out.println(isAtReference);
    return isAtReference;
  }

  public double getArmMotorAngularVelocity() {
    return breakMotor.getEncoder().getVelocity();
  }

  public double getRawArmPose() {
    return breakMotor.getEncoder().getPosition();
  }

  public double getArmPose() {
    double currPos = getRawArmPose();
    return convertRawArmPoseToArmPose(currPos);
  }

  public double getBreakMotorSpeed() {
    return breakMotor.getEncoder().getVelocity();
  }

  public boolean getExtentionStatus() {
    return extentionPiston.get();
  }

  public double getClawSpeedL() {
    return Math.abs(clawMotorL.getEncoder().getVelocity());
  }

  public double getClawSpeedR() {
    return Math.abs(clawMotorR.getEncoder().getVelocity());
  }

  public double getClawSpeed() {
    // average of two claws
    return (getClawSpeedL() + getClawSpeedR()) / 2.0;
  }

  public double getClawCurrentOutput() {
    return Math.copySign((Math.abs(clawMotorL.getOutputCurrent()) + Math.abs(clawMotorR.getOutputCurrent())) / 2.0,
        clawMotorL.getOutputCurrent() + clawMotorR.getOutputCurrent());
  }

  public boolean transferFromIntake(double clawSpeed) {
    setClawSpeed(clawSpeed);
    System.out.println(Math.abs(getClawCurrentOutput()));
    return Math.abs(getClawCurrentOutput()) > 15.0;

  }

}
