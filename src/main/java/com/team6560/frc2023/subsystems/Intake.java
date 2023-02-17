// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.team6560.frc2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANSparkMax leftExtensionMotor;
  private final CANSparkMax rightExtensionMotor;

  private final CANSparkMax[] extensionMotors;
  private final CANSparkMax rotationMotor;
  private GamePiece currentGamePiece = GamePiece.NONE;
  private boolean extended;

  public Intake() {
    leftExtensionMotor = new CANSparkMax(Constants.INTAKE_EXTENSION_MOTOR_LEFT, MotorType.kBrushless);
    rightExtensionMotor = new CANSparkMax(Constants.INTAKE_EXTENSION_MOTOR_RIGHT, MotorType.kBrushless);

    leftExtensionMotor.setInverted(false);
    rightExtensionMotor.setInverted(true);

    extensionMotors = new CANSparkMax[] {leftExtensionMotor, rightExtensionMotor};


    for (CANSparkMax i : extensionMotors) {
      i.setIdleMode(IdleMode.kBrake);

      i.getPIDController().setP(0.0);
      i.getPIDController().setI(0.0);
      i.getPIDController().setD(0.0);

      i.getPIDController().setIZone(0.0);
    }

    rotationMotor = new CANSparkMax(Constants.INTAKE_ROTATION_MOTOR, MotorType.kBrushless);
    rotationMotor.setIdleMode(IdleMode.kBrake);

    // rotationMotor.setSmartCurrentLimit(2);
    // rotationMotor.setSecondaryCurrentLimit(2);

    ntDispTab("Intake")
      .add("Rotation Motor Current Draw", this::getRotationMotorCurrent);
    
  }

  @Override
  public void periodic() {
    if (Math.abs(getRotationMotorCurrent()) > 25.0 && extended) {

      this.currentGamePiece = GamePiece.CONE;
    }

    if (getRotationMotorVelocity() > 100.0)
      this.currentGamePiece = GamePiece.NONE;

  }

  public void setIntakeOut(boolean extended) {
    this.extended = extended;
    if (extended) {
      setLeftExtensionMotorPosition(0.0);
      setRightExtensionMotorPosition(0.0);
    } else {
      setLeftExtensionMotorPosition(0.0);
      setRightExtensionMotorPosition(0.0);
    }
  }

  public boolean isSafeToRunRotationMotor() {
    if (currentGamePiece == GamePiece.CONE) {
      return false;
    }


    return true;
  }

  public void setLeftExtensionMotorPosition(double position) {
    leftExtensionMotor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public void setRightExtensionMotorPosition(double position) {
    rightExtensionMotor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public void setRotationMotor(double percentOutput) {
    rotationMotor.set(percentOutput);
  }

  public double getRotationMotorVelocity() {
    return rotationMotor.getEncoder().getVelocity();
  }

  public double getRotationMotorCurrent() {
    return rotationMotor.getOutputCurrent();
  }

  public GamePiece getCurrentGamePiece() {
    return currentGamePiece;
  }

}
