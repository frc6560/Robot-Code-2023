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
  CANSparkMax gripperMotor = new CANSparkMax(CLAW_ID, MotorType.kBrushless);
  
  Solenoid extentionPiston = new Solenoid(PneumaticsModuleType.CTREPCM, EXTENTION_SOLENOID_ID);

  NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");

  NetworkTableEntry breakMultiplyer;

  /** Creates a new Arm. */
  public Arm() {
    rotorMotor.restoreFactoryDefaults();
    rotorMotor.setInverted(true);
    rotorMotor.setIdleMode(IdleMode.kBrake);

    breakMotor.restoreFactoryDefaults();
    breakMotor.setIdleMode(IdleMode.kBrake);

    gripperMotor.restoreFactoryDefaults();
    gripperMotor.setIdleMode(IdleMode.kCoast);

    ntDispTab("Arm")
    .add("Rotor Speed", this::getRotorSpeed)
    .add("Break Motor Speed", this::getRotorSpeed)
    .add("Claw Speed", this::getRotorSpeed)
    .add("Extention Status", this::getRotorSpeed);

    breakMultiplyer = ntTable.getEntry("Break Motor Multiplyer");
    breakMultiplyer.setDouble(1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmExtention(boolean status){
    extentionPiston.set(status);
  }

  /** Sets arm rotor velocity in RPM */
  public void setRotors(double rpm){
    rotorMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
  }
  
  /** Sets arm break velocity in RPM */
  public void setBreakMotor(double rpm){
    double speed = rpm * breakMultiplyer.getDouble(1.0) * BREAK_MOTOR_MULTIPLIER;
    breakMotor.getPIDController().setReference(speed, ControlType.kVelocity);
  }

  public void setGripperRollers(double output){
    gripperMotor.set(output);
  }

  public void setArmRotation(double output){
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

  public double getClawSpeed(){
    return gripperMotor.getEncoder().getVelocity();
  }
}
