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
    *   move rotors & break motor
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
  NetworkTableEntry ntMarkRadin;

  NetworkTableEntry invertClaw;
  private double currentReference;


  public enum ArmPose {
    ZERO, DEFAULT, GROUND_CONE, LOW_CUBE, LOW_CONE, MEDIUM_CONE, HIGH_CONE, MEDIUM_CUBE, HIGH_CUBE, HUMAN_PLAYER_CONE, NONE, HUMAN_PLAYER_CUBE, GROUND_CUBE
  }

  public static HashMap<ArmPose, ArmState> armPoseMap = new HashMap<ArmPose, ArmState>();

  private static final double DEFAULT_TOP_SOFT_LIMIT = 121.0956969;
  private static final double ALLOWED_ERROR = 2.06560;

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
    .add("Target Pos", ()->currentReference)
    ;

    breakMultiplyer = ntTable.getEntry("Break Motor Multiplyer");
    breakMultiplyer.setDouble(0.1);

    ntTopLimit = ntTable.getEntry("Top Soft Limit");
    // ntTopLimit.setDouble(107.0);
    ntTopLimit.setDouble(DEFAULT_TOP_SOFT_LIMIT);
    ntBottomLimit = ntTable.getEntry("Bottom Soft Limit");
    ntBottomLimit.setDouble(0.0);

    invertClaw = ntTable.getEntry("Invert Claw?");
    invertClaw.setBoolean(false);

    ntMarkRadin = ntTable.getEntry("Mark (F) Radin (T)?"); // TODO: DELETE
    ntMarkRadin.setBoolean(false);

    // position, outSpeedMultiplier
    armPoseMap.put(ArmPose.ZERO, new ArmState(0.0, false, 1.0));

    // armPoseMap.put(ArmPose.DEFAULT, new Pair<Double, Double>(0.06321, 1.0));
    armPoseMap.put(ArmPose.DEFAULT, new ArmState(0.1, false, 1.0));

    armPoseMap.put(ArmPose.LOW_CUBE, new ArmState(0.2347, false, -2.0 * 0.175));
    armPoseMap.put(ArmPose.LOW_CONE, new ArmState(0.2347, false, -1.0 * 0.175));

    armPoseMap.put(ArmPose.GROUND_CUBE, new ArmState(0.3314, true, 0.5));
    armPoseMap.put(ArmPose.GROUND_CONE, new ArmState(0.3314, true, 1.0));

    armPoseMap.put(ArmPose.MEDIUM_CONE, new ArmState(0.770, false, -1.3 * 0.175));
    armPoseMap.put(ArmPose.HIGH_CONE, new ArmState(1.0, true, -1.0 * 0.175));

    armPoseMap.put(ArmPose.MEDIUM_CUBE, new ArmState(0.63, false, -2.0 * 0.175));
    armPoseMap.put(ArmPose.HIGH_CUBE, new ArmState(0.9, false, -2.0 * 0.175));

    armPoseMap.put(ArmPose.HUMAN_PLAYER_CUBE, new ArmState(0.81, false, 0.5));
    armPoseMap.put(ArmPose.HUMAN_PLAYER_CONE, new ArmState(0.81, false, 1.3));


    // armPidController.disableContinuousInput();
    // armPidController.setIntegratorRange(-0.5, 0.5);
    // armPidController.setTolerance(0.05);

    final double zeroToFullTime = 3;
    breakMotor.setOpenLoopRampRate(zeroToFullTime);

    breakMotorPid.setP(6.560e-8, 0);
    breakMotorPid.setI(1.06560e-9, 0);
    breakMotorPid.setD(6.560e-12, 0);
    breakMotorPid.setFF(0.001, 0);

    breakMotorPid.setSmartMotionMaxAccel(656.0, 0);
    breakMotorPid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    // breakMotorPid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    breakMotorPid.setSmartMotionMaxVelocity(1000, 0);
    // breakMotorPid.setSmartMotionMinOutputVelocity(50, 0);
    breakMotorPid.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR, 0);


    breakMotorPid.setP(0.05, 1);
    breakMotorPid.setI(1e-6, 1);
    breakMotorPid.setD(0, 1);
    breakMotorPid.setFF(0.000156, 1);
    
  }

  @Override
  public void periodic() {
  }

  public void setArmExtention(boolean status){
    System.out.println("Arm extended " + (status ? "Out." : "In."));

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
  public void setBreakMotor(double rpm){
    double speed = rpm * breakMultiplyer.getDouble(1.0) * BREAK_MOTOR_MULTIPLIER;

    // if(speed != 0) System.out.println("break is running at " + speed);
    
    breakMotor.set(-speed);
  }

  public void setClawSpeed(double output){
    if(output != 0) System.out.println("Claw is running at " + output);

    clawMotorL.set(output);
    clawMotorR.set(output);
  }

  public void setArmRotationVelocity(double output){
    if (getArmPose() <= armPoseMap.get(ArmPose.DEFAULT).getPosition()) {
      output = Math.min(0, output);

    } else if(getArmPose() >= 1.0) {
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
    // if(!ntMarkRadin.getBoolean(false)) setArmRotationMark(pose); else setArmRotationRadin(pose);
    if(!ntMarkRadin.getBoolean(false)) setArmRotation(pose); else setArmRotationRadin(pose);
    
  }

  public void setArmRotation(ArmPose armPose) {
    setArmRotation(armPoseMap.get(armPose).getPosition());
  }

  public void setArmRotation(double pose) {
    this.currentReference = pose;
    breakMotorPid.setReference(currentReference * (ntTopLimit.getDouble(DEFAULT_TOP_SOFT_LIMIT) - ntBottomLimit.getDouble(armPoseMap.get(ArmPose.ZERO).getPosition())), ControlType.kSmartMotion);
    // double calculated = -armPidController.calculate(getArmPose(), pose);
    // if (armPidController.atSetpoint())
    //   return;
    // setBreakMotorVolts(calculated);
  }

  public void setArmRotationRadin(double pose) {
    this.currentReference = pose;
    double rot = currentReference * (ntTopLimit.getDouble(DEFAULT_TOP_SOFT_LIMIT) - ntBottomLimit.getDouble(armPoseMap.get(ArmPose.ZERO).getPosition()));

    final double rampUpWindow = 750;
    if(getBreakMotorSpeed() < rampUpWindow && !isArmAtSetpoint()){
      setArmRotation(pose); // ramp up

    } else {
      breakMotorPid.setReference(rot, ControlType.kPosition, 1);
    } 

  }

  public void resetArmZero() {
    breakMotor.getEncoder().setPosition(0.0);
  }

  ProfiledPIDController controller = new ProfiledPIDController(breakMotorPid.getP(1), breakMotorPid.getI(1)*5, breakMotorPid.getD(1), new TrapezoidProfile.Constraints(11000, 4000));

  public void setArmRotationMark(double pose) {
    controller.disableContinuousInput();
    double ff = breakMotorPid.getFF(1);
    double volts = (controller.calculate(breakMotor.getEncoder().getPosition(), convertArmPoseToRawArmPose(pose)) + ff);
    System.out.println("Running marks at: " + volts);

    breakMotor.setVoltage(volts);
  }

  public boolean isArmAtSetpoint() {
    boolean isAtReference = Math.abs(currentReference - getArmPose()) < convertRawArmPoseToArmPose(ALLOWED_ERROR);
    System.out.println(isAtReference);
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

  public double getBreakMotorSpeed(){
    return breakMotor.getEncoder().getVelocity();
  }

  public boolean getExtentionStatus(){
    return extentionPiston.get();
  }

  public double getClawSpeedL(){
    return Math.abs(clawMotorL.getEncoder().getVelocity());
  }
  public double getClawSpeedR(){
    return Math.abs(clawMotorR.getEncoder().getVelocity());
  }
  public double getClawSpeed(){
    // average of two claws
    return (getClawSpeedL() + getClawSpeedR()) / 2.0;
  }

  

}
