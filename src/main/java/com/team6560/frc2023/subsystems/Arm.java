// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
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
  CANSparkMax clawMotorL = new CANSparkMax(CLAW_MOTOR_LEFT_ID, MotorType.kBrushless);
  CANSparkMax clawMotorR = new CANSparkMax(CLAW_MOTOR_RIGHT_ID, MotorType.kBrushless);
  
  Solenoid extentionPiston = new Solenoid(PneumaticsModuleType.CTREPCM, EXTENTION_SOLENOID_ID);

  NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");

  NetworkTableEntry breakMultiplyer;

  NetworkTableEntry ntTopLimit;
  NetworkTableEntry ntBottomLimit;

  NetworkTableEntry invertClaw;


  public enum ArmPose {
    ZERO, GROUND, TRANSFER, LOW, MEDIUM_CONE, HIGH_CONE, MEDIUM_CUBE, HIGH_CUBE, HUMAN_PLAYER, TRANSFER_PART_2
  }

  private HashMap<ArmPose, Pair<Double, Boolean>> armPoseMap = new HashMap<ArmPose, Pair<Double, Boolean>>();
  private int frames;


  // private PIDController armPidController = new PIDController(25.0, 7.25, 6.0);

  /** Creates a new Arm. */
  public Arm() {
    breakMotor.restoreFactoryDefaults();
    // breakMotor.setInverted(true);
    breakMotor.setIdleMode(IdleMode.kBrake);

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
    ;

    breakMultiplyer = ntTable.getEntry("Break Motor Multiplyer");
    breakMultiplyer.setDouble(0.1);

    ntTopLimit = ntTable.getEntry("Top Soft Limit");
    // ntTopLimit.setDouble(107.0);
    ntTopLimit.setDouble(112.3);
    ntBottomLimit = ntTable.getEntry("Bottom Soft Limit");
    ntBottomLimit.setDouble(0.0);

    invertClaw = ntTable.getEntry("Invert Claw?");
    invertClaw.setBoolean(false);


    armPoseMap.put(ArmPose.ZERO, new Pair<Double, Boolean>(0.1, false));
    armPoseMap.put(ArmPose.LOW, new Pair<Double, Boolean>(0.1, false));
    armPoseMap.put(ArmPose.GROUND, new Pair<Double, Boolean>(0.2, true));

    armPoseMap.put(ArmPose.MEDIUM_CONE, new Pair<Double, Boolean>(0.5, false));
    armPoseMap.put(ArmPose.HIGH_CONE, new Pair<Double, Boolean>(0.1, true));

    armPoseMap.put(ArmPose.MEDIUM_CUBE, new Pair<Double, Boolean>(0.1, false));
    armPoseMap.put(ArmPose.HIGH_CUBE, new Pair<Double, Boolean>(0.1, true));

    armPoseMap.put(ArmPose.HUMAN_PLAYER, new Pair<Double, Boolean>(1.0, false));

    armPoseMap.put(ArmPose.TRANSFER, new Pair<Double, Boolean>(0.3, false));

    // armPidController.disableContinuousInput();
    // armPidController.setIntegratorRange(-0.5, 0.5);
    // armPidController.setTolerance(0.05);

    breakMotor.getPIDController().setP(6.560e-8, 0);
    breakMotor.getPIDController().setI(1e-9, 0);
    breakMotor.getPIDController().setD(0.0, 0);
    breakMotor.getPIDController().setFF(0.00156560, 0);

    breakMotor.getPIDController().setSmartMotionMaxAccel(750.0, 0);
    breakMotor.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
    breakMotor.getPIDController().setSmartMotionMaxVelocity(10000, 0);
    // breakMotor.getPIDController().setSmartMotionMinOutputVelocity(50, 0);
    breakMotor.getPIDController().setSmartMotionAllowedClosedLoopError(2.0, 0);
  }

  @Override
  public void periodic() {
  }

  public boolean transferFromIntake() {
    setArmState(ArmPose.TRANSFER);
    setClawSpeed(0.4);
    return Math.abs(getClawCurrentDraw()) > 20;
  }

  public boolean transferFromIntakePart2() {
    setArmState(ArmPose.TRANSFER_PART_2);
    setClawSpeed(0.0);
    return frames++ > 20;
  }

  public double getClawCurrentDraw() {
    return Math.copySign(Math.abs(clawMotorL.getOutputCurrent()) + Math.abs(clawMotorR.getOutputCurrent()), clawMotorL.getOutputCurrent() + clawMotorR.getOutputCurrent()) / 2.0;
  }

  public void setArmExtention(boolean status){
    System.out.println("Arm extended " + (status ? "Out." : "In."));

    extentionPiston.set(status);
  }

  public double getRawArmPose() {
    return breakMotor.getEncoder().getPosition();
  }

  public double getArmPose() {
    double currPos = getRawArmPose();
    double low = ntBottomLimit.getDouble(0.0);
    // double high = ntTopLimit.getDouble(107.0);
    double high = ntTopLimit.getDouble(112.3);

    return (currPos - low) / (high - low);
  }
  
  /** Sets arm break velocity in RPM */
  public void setBreakMotor(double rpm){
    double speed = rpm * breakMultiplyer.getDouble(1.0) * BREAK_MOTOR_MULTIPLIER;

    if(speed != 0) System.out.println("break is running at " + speed);
    
    breakMotor.set(-speed);
  }

  public void setBreakMotorVolts(double volts) {
    breakMotor.setVoltage(volts);
  }

  public void setClawSpeed(double output){
    if(output != 0) System.out.println("Claw is running at " + output);

    clawMotorL.set(output);
    clawMotorR.set(output);
  }

  public void setArmRotationVelocity(double output){

    if (getArmPose() <= 0) {
      output = Math.min(0, output);

    } else if(getArmPose() >= 1.0) {
      output = Math.max(0, output);
    }

    setBreakMotor(output * BREAK_TO_ARM);
  }


  public void setArmState(ArmPose armPose) {
    setArmState(armPoseMap.get(armPose));
  }


  public void setArmState(Pair<Double, Boolean> state) {
    setArmState(state.getFirst(), state.getSecond());
  }

  public void setArmState(double pose, boolean extended) {
    setArmRotation(pose);
    setArmExtention(extended);
  }

  public void setArmRotation(ArmPose armPose) {
    setArmRotation(armPoseMap.get(armPose).getFirst());
  }

  public void setArmRotation(double pose) {
    breakMotor.getPIDController().setReference(pose * (ntTopLimit.getDouble(112.3) - ntBottomLimit.getDouble(0.0)), ControlType.kSmartMotion);
    // double calculated = -armPidController.calculate(getArmPose(), pose);
    // if (armPidController.atSetpoint())
    //   return;
    // setBreakMotorVolts(calculated);
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
