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

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import java.util.HashMap;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public enum ArmPose {
    ZERO, DEFAULT, GROUND_CONE, LOW_CUBE, LOW_CONE, MEDIUM_CONE, HIGH_CONE, MEDIUM_CUBE, HIGH_CUBE, HUMAN_PLAYER_CONE,
    NONE, HUMAN_PLAYER_CUBE, GROUND_CUBE, INTAKE_CONE, INTAKE_CUBE, CLEARANCE
  }

  public static HashMap<ArmPose, ArmState> armPoseMap = new HashMap<ArmPose, ArmState>();

  private static final double DEFAULT_TOP_SOFT_LIMIT = 235.0;
  public static final double ALLOWED_ERROR = 4.5;

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
    clawMotorL.setInverted(true);

    ntDispTab("Arm")
        .add("Break Motor Speed", this::getBreakMotorSpeed)
        .add("Claw Speed Left", this::getClawSpeedL)
        .add("Claw Speed Right", this::getClawSpeedR)
        .add("Claw current", this::getClawCurrentOutput)
        .add("target current", ()->20)
        .add("Extention Status", this::getExtentionStatus)
        .add("raw arm pos", this::getRawArmPose)
        .add("armPose", this::getArmPose)
        .add("Target Pos", () -> currentReference)
        .add("LeftCurrent", () -> this.clawMotorL.getOutputCurrent())
        .add("RightCurrent", () -> this.clawMotorR.getOutputCurrent());

    breakMultiplyer = ntTable.getEntry("Break Motor Multiplyer");
    breakMultiplyer.setDouble(0.1);

    ntTopLimit = ntTable.getEntry("Top Soft Limit");
    // ntTopLimit.setDouble(107.0);
    ntTopLimit.setDouble(DEFAULT_TOP_SOFT_LIMIT);
    ntBottomLimit = ntTable.getEntry("Bottom Soft Limit");
    ntBottomLimit.setDouble(0.0);

    invertClaw = ntTable.getEntry("Invert Claw?");
    invertClaw.setBoolean(false);


    // position, outSpeedMultiplier
    armPoseMap.put(ArmPose.ZERO, new ArmState(0.0, false, 1.0));

    // armPoseMap.put(ArmPose.DEFAULT, new Pair<Double, Double>(0.06321, 1.0));
    armPoseMap.put(ArmPose.DEFAULT, new ArmState(0.1, false, 1.0));

    armPoseMap.put(ArmPose.LOW_CUBE, new ArmState(0.245, false, 2.5 * 0.175));
    armPoseMap.put(ArmPose.LOW_CONE, new ArmState(0.245, false, 1.0 * 0.175));

    armPoseMap.put(ArmPose.GROUND_CUBE, new ArmState(0.37, true, 1.0));
    armPoseMap.put(ArmPose.GROUND_CONE, new ArmState(0.35, true, 1.0));

    armPoseMap.put(ArmPose.MEDIUM_CONE, new ArmState(0.79, false, 1.4 * 0.175));
    armPoseMap.put(ArmPose.HIGH_CONE, new ArmState(0.98, true, 1.25 * 0.175));

    armPoseMap.put(ArmPose.MEDIUM_CUBE, new ArmState(0.7, false, 2.0 * 0.175));
    armPoseMap.put(ArmPose.HIGH_CUBE, new ArmState(0.9, true, 2.0 * 0.175));

    armPoseMap.put(ArmPose.HUMAN_PLAYER_CUBE, new ArmState(0.804, false, 0.85));
    armPoseMap.put(ArmPose.HUMAN_PLAYER_CONE, new ArmState(0.798, false, 1.3));

    armPoseMap.put(ArmPose.INTAKE_CONE, new ArmState(0.36, false, 1.3));
    armPoseMap.put(ArmPose.INTAKE_CUBE, new ArmState(0.145, false, 0.5));

    armPoseMap.put(ArmPose.CLEARANCE, new ArmState(IntakeConstants.ROTATION_ARM_CLEARANCE, false, 1.0));
    // armPidController.disableContinuousInput();
    // armPidController.setIntegratorRange(-0.5, 0.5);
    // armPidController.setTolerance(0.05);

    final double zeroToFullTime = 0.35;
    breakMotor.setOpenLoopRampRate(zeroToFullTime);

    breakMotorPid.setP(6.560e-8, 0);
    breakMotorPid.setI(1.06560e-9, 0);
    breakMotorPid.setD(6.560e-12, 0);
    breakMotorPid.setFF(0.003, 0);

    breakMotorPid.setSmartMotionMaxAccel(600.0, 0);
    breakMotorPid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    // breakMotorPid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    breakMotorPid.setSmartMotionMaxVelocity(9500, 0);
    // breakMotorPid.setSmartMotionMinOutputVelocity(50, 0);
    // breakMotorPid.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR, 0);

    breakMotorPid.setP(0.025, 1);
    breakMotorPid.setI(1e-7, 1);
    breakMotorPid.setD(0, 1);
    breakMotorPid.setFF(0.00008, 1);

    breakMotor.setSmartCurrentLimit(25);
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
    double high = ntTopLimit.getDouble(DEFAULT_TOP_SOFT_LIMIT);

    return (rawArmPose - low) / (high - low);
  }

  public double convertArmPoseToRawArmPose(double armPose) {
    double low = ntBottomLimit.getDouble(0.0);
    double high = ntTopLimit.getDouble(DEFAULT_TOP_SOFT_LIMIT);

    return armPose * (high - low) + low;
  }

  /** Sets arm break velocity in RPM */
  public void setBreakMotor(double rpm) {
    double speed = rpm * breakMultiplyer.getDouble(1.0) * BREAK_MOTOR_MULTIPLIER;

    // if(speed != 0) System.out.println("break is running at " + speed);

    breakMotor.set(-speed);
  }

  boolean hasPiece = false;

  public void setClawSpeed(double output) {
    clawMotorL.set(output);
    clawMotorR.set(output);
  }

  public void setArmRotationVelocity(double output) {
    if (getArmPose() <= armPoseMap.get(ArmPose.DEFAULT).getPosition()) {
      output = Math.min(0, output);

    } else if (getArmPose() >= 1.0) {
      output = Math.max(0, output);
    }

    setBreakMotor(output * BREAK_TO_ARM);
  }

  public void setArmRotationVelocityOverrideSoftLimits(double output) {
    setBreakMotor(output * BREAK_TO_ARM);
  }

  public void setArmState(ArmPose armPose) {
    setArmState(armPoseMap.get(armPose).getPosition());
  }

  public void setArmState(double pose) {
    // setArmRotation(pose);
    // if(!ntMarkRadin.getBoolean(false)) setArmRotationMark(pose); else
    // setArmRotationRadin(pose);
    setArmRotation(pose);

  }

  public void setArmRotation(ArmPose armPose) {
    setArmRotation(armPoseMap.get(armPose).getPosition());
  }

  public void setArmRotationOld(double pose) {
    this.currentReference = pose;
    breakMotorPid.setReference(currentReference * (ntTopLimit.getDouble(DEFAULT_TOP_SOFT_LIMIT)
        - ntBottomLimit.getDouble(armPoseMap.get(ArmPose.ZERO).getPosition())), ControlType.kSmartMotion);
    // double calculated = -armPidController.calculate(getArmPose(), pose);
    // if (armPidController.atSetpoint())
    // return;
    // setBreakMotorVolts(calculated);
  }

  public void setArmRotation(double pose) {
    this.currentReference = pose;
    double rot = currentReference * (ntTopLimit.getDouble(DEFAULT_TOP_SOFT_LIMIT)
        - ntBottomLimit.getDouble(armPoseMap.get(ArmPose.ZERO).getPosition()));

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

  public boolean isArmAtSetpoint() {
    boolean isAtReference = Math.abs(currentReference - getArmPose()) < convertRawArmPoseToArmPose(ALLOWED_ERROR);
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
    return (Math.abs(clawMotorL.getOutputCurrent()) + Math.abs(clawMotorR.getOutputCurrent())) / 2.0;
  }

  public boolean hasCube() {
    return Math.abs(getClawSpeedR()) > 2500 && Math.abs(getClawCurrentOutput()) > 12.5;
  }
  public boolean hasCone() {
    return Math.abs(getClawSpeedR()) > 1500 && Math.abs(getClawCurrentOutput()) > 5.5;
  }

  public boolean transferFromIntake(double clawSpeed) {
    setClawSpeed(clawSpeed);
    System.out.println(Math.abs(getClawCurrentOutput()));
    return hasCube();

  }
  
  public boolean canRunIntake(){
    return Math.abs(getArmPose() - IntakeConstants.ROTATION_ARM_CLEARANCE) < convertRawArmPoseToArmPose(Arm.ALLOWED_ERROR);
  }

}
