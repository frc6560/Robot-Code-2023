// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team6560.frc2023.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANSparkMax leftExtensionMotor;
  private final CANSparkMax rightExtensionMotor;

  private final CANSparkMax[] extensionMotors;
  private final CANSparkMax rotationMotor;
  private GamePiece currentGamePiece = GamePiece.NONE;

  private HashMap<IntakeState, Double> intakeStateMap;
  private IntakeState currIntakeState;

  public enum IntakeState {
    EXTENDED, RETRACTED
  }

  public Intake() {
    leftExtensionMotor = new CANSparkMax(Constants.INTAKE_EXTENSION_MOTOR_LEFT, MotorType.kBrushless);
    rightExtensionMotor = new CANSparkMax(Constants.INTAKE_EXTENSION_MOTOR_RIGHT, MotorType.kBrushless);

    // leftExtensionMotor.setInverted(false);
    // rightExtensionMotor.setInverted(true);

    extensionMotors = new CANSparkMax[] {leftExtensionMotor, rightExtensionMotor};

    intakeStateMap = new HashMap<IntakeState, Double>();

    for (CANSparkMax i : extensionMotors) {
      i.restoreFactoryDefaults();

      i.setIdleMode(IdleMode.kBrake);

      i.getPIDController().setP(6.560e-9, 0);
      i.getPIDController().setI(1.06560e-10, 0);
      i.getPIDController().setD(6.560e-12, 0);
      i.getPIDController().setFF(0.001, 0);

      i.getPIDController().setSmartMotionMaxAccel(300, 0);
      // i.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
      i.getPIDController().setSmartMotionMaxVelocity(5000, 0);
      // i.getPIDController().setSmartMotionMinOutputVelocity(50, 0);
      i.getPIDController().setSmartMotionAllowedClosedLoopError(0.5, 0);
    }

    rotationMotor = new CANSparkMax(Constants.INTAKE_ROTATION_MOTOR, MotorType.kBrushless);
    rotationMotor.setIdleMode(IdleMode.kBrake);

    // rotationMotor.setSmartCurrentLimit(2);
    // rotationMotor.setSecondaryCurrentLimit(2);

    ntDispTab("Intake")
    .add("Rotation Motor Current Draw", this::getRotationMotorCurrent)
    .add("Intake extension pose", () -> leftExtensionMotor.getEncoder().getPosition());
    
      intakeStateMap.put(IntakeState.EXTENDED, 1.0);
      intakeStateMap.put(IntakeState.RETRACTED, 0.0);

    }

  @Override
  public void periodic() {
    if (Math.abs(getRotationMotorCurrent()) > 25.0 && currIntakeState == IntakeState.EXTENDED) {

      this.currentGamePiece = GamePiece.CONE;
    }

    if (getRotationMotorVelocity() > 100.0)
      this.currentGamePiece = GamePiece.NONE;

  }


  public void setIntakeState(IntakeState intakeState) {
    this.currIntakeState = intakeState;
    setLeftExtensionMotorPosition(intakeStateMap.get(intakeState));
    setRightExtensionMotorPosition(-intakeStateMap.get(intakeState));
  }

  public boolean isSafeToRunRotationMotor() {
    if (currentGamePiece == GamePiece.CONE) {
      return false;
    }


    return true;
  }

  public IntakeState getCurrentState() {
    if (Math.abs(leftExtensionMotor.getEncoder().getPosition()) < 1.0)
      return IntakeState.RETRACTED;
    return IntakeState.EXTENDED;
  }

  public static double convertPositionToPercent(double position) {
    double high = 3.6;
    double low = 0;
    return (position - low) / (high - low);

  }

  public void setLeftExtensionMotorPosition(double position) {
    leftExtensionMotor.getPIDController().setReference(convertPositionToPercent(position), ControlType.kSmartMotion);
  }

  public void setRightExtensionMotorPosition(double position) {
    rightExtensionMotor.getPIDController().setReference(convertPositionToPercent(position), ControlType.kSmartMotion);
  }


  public void moveIntake(double percentOutput) {
    leftExtensionMotor.set(-percentOutput);
    rightExtensionMotor.set(percentOutput);
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
