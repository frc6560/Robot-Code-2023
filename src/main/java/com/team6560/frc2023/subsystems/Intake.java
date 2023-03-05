// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.team6560.frc2023.Constants;
import com.team6560.frc2023.subsystems.Arm.ArmPose;
import com.team6560.frc2023.utility.NetworkTable.NtValueDisplay;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax leftIntakeMotor;
  private CANSparkMax rightIntakeMotor;
  private CANSparkMax intakeSuckMotor;

  public static final double MAX_OUTAKE_POSITION = 16.5;

  public static enum SuckState {
    SUCK, OUTAKE, NOTHING
  }

  public static enum IntakePose {
    EXTENDED_CUBE, EXTENDED_CONE, HANDOFF_CONE, HANDOFF_CUBE, RETRACTED,
  }

  public HashMap<IntakePose, IntakeState> intakePoseMap = new HashMap<IntakePose, IntakeState>();
  private IntakeState currSetIntakeState = new IntakeState(0.0, 0.0, ArmPose.NONE);
  private IntakePose currSetIntakePose = IntakePose.RETRACTED;
  private boolean inverted;

  /** Creates a new Intake. */
  public Intake() {
    this.leftIntakeMotor = new CANSparkMax(Constants.INTAKE_EXTENSION_MOTOR_LEFT, MotorType.kBrushless);
    this.rightIntakeMotor = new CANSparkMax(Constants.INTAKE_EXTENSION_MOTOR_RIGHT, MotorType.kBrushless);

    this.intakeSuckMotor = new CANSparkMax(Constants.INTAKE_ROTATION_MOTOR, MotorType.kBrushless);
    

  
    rightIntakeMotor.restoreFactoryDefaults();
    rightIntakeMotor.getEncoder().setPosition(0.255);

    SparkMaxPIDController pid = rightIntakeMotor.getPIDController();

    pid.setP(1e-5, 0);
    pid.setI(0, 0);
    pid.setD(0, 0);
    pid.setFF(0.002, 0);

    pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    pid.setSmartMotionMaxAccel(750, 0);
    pid.setSmartMotionMaxVelocity(1600, 0);
    pid.setSmartMotionAllowedClosedLoopError(0.02, 0);
    

    leftIntakeMotor.follow(rightIntakeMotor, true);
    
    rightIntakeMotor.getEncoder().setPositionConversionFactor(1.0 / MAX_OUTAKE_POSITION);
    // rightIntakeMotor.getEncoder().setVelocityConversionFactor(1);

    NtValueDisplay.ntDispTab("Intake")
      .add("Position", () -> this.rightIntakeMotor.getEncoder().getPosition());
    
      intakePoseMap.put(IntakePose.EXTENDED_CUBE, new IntakeState(1.0, -0.5, ArmPose.DEFAULT));
      intakePoseMap.put(IntakePose.EXTENDED_CONE, new IntakeState(1.0, -0.5, ArmPose.DEFAULT));
      intakePoseMap.put(IntakePose.RETRACTED, new IntakeState(0.0, 0.0, ArmPose.NONE));
      intakePoseMap.put(IntakePose.HANDOFF_CUBE, new IntakeState(1.0, 0.0, ArmPose.INTAKE_CUBE));
      intakePoseMap.put(IntakePose.HANDOFF_CONE, new IntakeState(1.0, 0.0, ArmPose.INTAKE_CONE));
    }

  public void setIntakePosition(double intakePosition) {
    rightIntakeMotor.getPIDController().setReference(intakePosition, ControlType.kSmartMotion, 0);
  }

  private void setIntakeState(IntakeState intakeState) {
    this.currSetIntakeState = intakeState;
  }

  public void setIntakeState(IntakePose intakePose) {
    setIntakeState(intakePoseMap.get(intakePose));
    this.currSetIntakePose = intakePose;
  }

  public void setSuckMotor(double velocity) {
    velocity = inverted ? -velocity : velocity;
    intakeSuckMotor.set(velocity);
  }

  public void setSuckMotor(SuckState suckState) {
    switch (suckState) {
      case NOTHING:
        intakeSuckMotor.set(0.0);
        break;
      case SUCK:
        intakeSuckMotor.set(0.5);
        break;
      case OUTAKE:
        intakeSuckMotor.set(-0.5);
        break;
      default:
        intakeSuckMotor.set(0.0);
        break;
    }
  }

  public double getIntakePosition() {
    return rightIntakeMotor.getEncoder().getPosition();
  }

  public double getCurrentDraw() {
    return rightIntakeMotor.getOutputCurrent();
  }

  public boolean hasObject() {
    return Math.abs(getCurrentDraw()) > 10.0;
  }

  public boolean atSetpoint() {
    return Math.abs(getIntakePosition() - currSetIntakeState.getPosition()) < rightIntakeMotor.getPIDController().getSmartMotionAllowedClosedLoopError(0);
  }

  public IntakeState getCurrentState() {
    return currSetIntakeState;
  }

  public IntakePose getCurrentPose() {
    return currSetIntakePose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setIntakePosition(currSetIntakeState.getPosition());
    if (Math.abs(currSetIntakeState.getPosition() - rightIntakeMotor.getEncoder().getPosition()) < rightIntakeMotor.getPIDController().getSmartMotionAllowedClosedLoopError(0)) {
      setSuckMotor(currSetIntakeState.getSuckSpeed());
    }
  }

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }
}
