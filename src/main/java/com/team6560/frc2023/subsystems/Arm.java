// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import javax.swing.text.StyledEditorKit.BoldAction;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
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

  CANSparkMax rotorMotor = new CANSparkMax(ROTOR_ID, MotorType.kBrushless);
  CANSparkMax breakMotor = new CANSparkMax(BREAK_ID, MotorType.kBrushless);
  CANSparkMax clawMotorL = new CANSparkMax(CLAW_MOTOR_LEFT_ID, MotorType.kBrushless);
  CANSparkMax clawMotorR = new CANSparkMax(CLAW_MOTOR_RIGHT_ID, MotorType.kBrushless);
  
  Solenoid extentionPiston = new Solenoid(PneumaticsModuleType.CTREPCM, EXTENTION_SOLENOID_ID);

  NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");

  NetworkTableEntry breakMultiplyer;

  NetworkTableEntry ntTopLimit;
  NetworkTableEntry ntBottomLimit;

  NetworkTableEntry invertClaw;

  private double topLimit = 0;
  private double bottomLimit = 0;
  /** Creates a new Arm. */
  public Arm() {
    rotorMotor.restoreFactoryDefaults();
    rotorMotor.setInverted(false);
    rotorMotor.setIdleMode(IdleMode.kBrake);
    // rotorMotor.getPIDController().setP(0.5);

    rotorMotor.getEncoder().setPosition(0.0);

    breakMotor.restoreFactoryDefaults();
    breakMotor.setIdleMode(IdleMode.kBrake);
    // breakMotor.getPIDController().setP(0.5);

    clawMotorL.restoreFactoryDefaults();
    clawMotorL.setIdleMode(IdleMode.kCoast);
    clawMotorL.setInverted(true);
    
    clawMotorR.restoreFactoryDefaults();
    clawMotorR.setIdleMode(IdleMode.kCoast);

    ntDispTab("Arm")
    .add("Rotor Speed", this::getRotorSpeed)
    .add("Break Motor Speed", this::getBreakMotorSpeed)
    .add("Claw Speed Left", this::getClawSpeedL)
    .add("Claw Speed Right", this::getClawSpeedR)
    .add("Extention Status", this::getExtentionStatus)
    .add("Arm Position", this::getArmPosition);

    breakMultiplyer = ntTable.getEntry("Break Motor Multiplyer");
    breakMultiplyer.setDouble(0.1);

    ntTopLimit = ntTable.getEntry("Top Soft Limit");
    ntTopLimit.setDouble(180.0);
    ntBottomLimit = ntTable.getEntry("Bottom Soft Limit");
    ntBottomLimit.setDouble(10.0);

    invertClaw = ntTable.getEntry("Invert Claw?");
    invertClaw.setBoolean(false);
  }

  @Override
  public void periodic() {
    topLimit = ntTopLimit.getDouble(0.0);
    bottomLimit = ntBottomLimit.getDouble(0.0);
    // This method will be called once per scheduler run
  }

  public void setArmExtention(boolean status){
    System.out.println("Arm extended " + (status ? "Out." : "In."));

    extentionPiston.set(status);
  }

  /** Sets arm rotor velocity in RPM */
  public void setRotors(double rpm){
    if(rpm != 0) System.out.println("rotor is running at " + rpm);

    rotorMotor.set(rpm);
  }
  
  /** Sets arm break velocity in RPM */
  public void setBreakMotor(double rpm){
    double speed = rpm * breakMultiplyer.getDouble(1.0) * BREAK_MOTOR_MULTIPLIER;

    if(speed != 0) System.out.println("break is running at " + speed);
    
    breakMotor.set(speed);
  }

  public void setClawSpeed(double output){
    if(output != 0) System.out.println("Claw is running at " + output);

    clawMotorL.set(output * (invertClaw.getBoolean(false) ? -1 : 1));
    clawMotorR.set(output * (invertClaw.getBoolean(false) ? -1 : 1));
  }

  public void setArmRotation(double output){
    System.out.println("Tryna rotate arm at " + output);

    // if (getArmPositionDegrees() < bottomLimit) {
    //   output = Math.min(0, output);

    // } else if(getArmPositionDegrees() > topLimit) {
    //   output = Math.max(0, output);
    // }

    setRotors(output * ROTOR_TO_ARM);
    setBreakMotor(output * BREAK_TO_ARM);
  }


  public double getRotorSpeed(){
    return rotorMotor.getEncoder().getVelocity();
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

  public double getArmPosition(){
    return rotorMotor.getEncoder().getPosition() / ROTOR_TO_ARM;
  }
  public double getArmPositionDegrees(){
    return rotToDeg(getArmPosition());
  }

  // gives rotation to degrees cuz im too lazy
  private double rotToDeg(double rotation){
    return rotation * 360;
  }
}
