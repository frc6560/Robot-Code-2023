// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import java.util.HashMap;

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
    ZERO, LOW, MEDIUM_CONE, HIGH_CONE, MEDIUM_CUBE, HIGH_CUBE, HUMAN_PLAYER
  }

  private HashMap<ArmPose, Double> armPoseMap = new HashMap<ArmPose, Double>();


  private PIDController armPidController = new PIDController(0.0, 0.0, 0.0);

  /** Creates a new Arm. */
  public Arm() {
    breakMotor.restoreFactoryDefaults();
    breakMotor.setInverted(true);
    breakMotor.setIdleMode(IdleMode.kBrake);
    // breakMotor.getPIDController().setP(0.5);

    clawMotorL.restoreFactoryDefaults();
    clawMotorL.setIdleMode(IdleMode.kCoast);
    
    clawMotorR.restoreFactoryDefaults();
    clawMotorR.setIdleMode(IdleMode.kCoast);
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
    ntTopLimit.setDouble(120.0);
    ntBottomLimit = ntTable.getEntry("Bottom Soft Limit");
    ntBottomLimit.setDouble(0.0);

    invertClaw = ntTable.getEntry("Invert Claw?");
    invertClaw.setBoolean(false);


    armPoseMap.put(ArmPose.ZERO, 0.0);
    armPoseMap.put(ArmPose.LOW, 0.1);

    armPoseMap.put(ArmPose.MEDIUM_CONE, 0.1);
    armPoseMap.put(ArmPose.HIGH_CONE, 0.1);

    armPoseMap.put(ArmPose.MEDIUM_CUBE, 0.1);
    armPoseMap.put(ArmPose.HIGH_CUBE, 0.1);

    armPoseMap.put(ArmPose.HUMAN_PLAYER, 1.0);


    armPidController.disableContinuousInput();
    // armPidController.setIntegratorRange(0.0, 0.0);
    armPidController.setTolerance(0.0);
  }

  @Override
  public void periodic() {
  }

  public void setArmExtention(boolean status){
    System.out.println("Arm extended " + (status ? "Out." : "In."));

    extentionPiston.set(status);
  }

  public double getRawArmPose() {
    return -breakMotor.getEncoder().getPosition();
  }

  public double getArmPose() {
    double currPos = getRawArmPose();
    double low = ntBottomLimit.getDouble(0.0);
    double high = ntTopLimit.getDouble(120.0);

    if (currPos < low) return 0.0;
    if (currPos > high) return 1.0;

    return (currPos - low) / (high - low);
  }
  
  /** Sets arm break velocity in RPM */
  public void setBreakMotor(double rpm){
    double speed = rpm * breakMultiplyer.getDouble(1.0) * BREAK_MOTOR_MULTIPLIER;

    if(speed != 0) System.out.println("break is running at " + speed);
    
    breakMotor.set(speed);
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

    if (getArmPose() < 0) {
      output = Math.min(0, output);

    } else if(getArmPose() > 1.0) {
      output = Math.max(0, output);
    }

    setBreakMotor(output * BREAK_TO_ARM);
  }


  public void setArmRotation(ArmPose armPose) {
    setArmRotation(armPoseMap.get(armPose));
  }

  public void setArmRotation(double pose) {
    setBreakMotorVolts(armPidController.calculate(getArmPose(), pose));
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
