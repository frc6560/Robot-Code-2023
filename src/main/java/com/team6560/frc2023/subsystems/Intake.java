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
import com.team6560.frc2023.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANSparkMax leftExtensionMotor;
  private final CANSparkMax rightExtensionMotor;

  private final CANSparkMax[] extensionMotors;
  private final CANSparkMax rotationMotor;

  private HashMap<IntakeState, Double> intakeStateMap;

  private final double OBJECT_CURRENT = 25.0;
  private boolean reverseMotor = false;

  private IntakeState targetState;


  public enum IntakeState {
    EXTENDED_CONE, EXTENDED_CUBE, RETRACTED, HANDOFF_CONE
  }

  public Intake() {
    leftExtensionMotor = new CANSparkMax(Constants.INTAKE_EXTENSION_MOTOR_LEFT, MotorType.kBrushless);
    rightExtensionMotor = new CANSparkMax(Constants.INTAKE_EXTENSION_MOTOR_RIGHT, MotorType.kBrushless);

    // leftExtensionMotor.setInverted(false);
    // rightExtensionMotor.setInverted(true);

    extensionMotors = new CANSparkMax[] { leftExtensionMotor, rightExtensionMotor };

    intakeStateMap = new HashMap<IntakeState, Double>();

    for (CANSparkMax i : extensionMotors) {
      i.restoreFactoryDefaults();

      i.setIdleMode(IdleMode.kBrake);

      i.getPIDController().setP(5e-3, 0);
      i.getPIDController().setI(1.0e-6, 0);
      i.getPIDController().setD(0, 0);
      i.getPIDController().setFF(0.001, 0);

      i.getPIDController().setSmartMotionMaxAccel(300, 0);
      // i.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,
      // 0);
      i.getPIDController().setSmartMotionMaxVelocity(5000, 0);
      // i.getPIDController().setSmartMotionMinOutputVelocity(50, 0);
      i.getPIDController().setSmartMotionAllowedClosedLoopError(0.5, 0);
    }

    rotationMotor = new CANSparkMax(Constants.INTAKE_ROTATION_MOTOR, MotorType.kBrushless);
    rotationMotor.setIdleMode(IdleMode.kBrake);

    ntDispTab("Intake")
        .add("Rotation Motor Current Draw", this::getFeedMotorCurrent)
        .add("Intake extension pose", this::getIntakePosition);

    intakeStateMap.put(IntakeState.EXTENDED_CONE, 1.0);
    intakeStateMap.put(IntakeState.EXTENDED_CUBE, 0.85);
    intakeStateMap.put(IntakeState.RETRACTED, 0.0);
    intakeStateMap.put(IntakeState.HANDOFF_CONE, 0.75);

    leftExtensionMotor.getEncoder().setPosition(-5.0);

    targetState = IntakeState.RETRACTED;
  }

  @Override
  public void periodic() {
    setIntakeState(targetState);
  }


  public void setIntakeState(IntakeState intakeState){
    targetState = intakeState;
  }

  public void setIntake(IntakeState intakeState) {
    setLeftExtensionMotorPosition(intakeStateMap.get(intakeState));
    setRightExtensionMotorPosition(-intakeStateMap.get(intakeState));

    double output;

    if (intakeState == IntakeState.EXTENDED_CONE){
      output = IntakeConstants.INTAKE_CONE_FEED_RPM;

    } else if(intakeState == IntakeState.EXTENDED_CUBE){
      output = IntakeConstants.INTAKE_CUBE_FEED_RPM;

    } else if(intakeState == IntakeState.HANDOFF_CONE){
      output = IntakeConstants.HANDOFF_SPEED;

    } else{
      output = 0.0;
    }

    if(reverseMotor){
      output *= -1;
    } else if((intakeState == IntakeState.EXTENDED_CONE ||  intakeState == IntakeState.EXTENDED_CUBE) && hasPiece()){
      output = 0;
    }

    setFeedMotor(output);
  }

  public void setLeftExtensionMotorPosition(double position) {
    System.out.println(position);
    leftExtensionMotor.getPIDController().setReference(convertPositionToPercent(position), ControlType.kSmartMotion);
  }

  public void setRightExtensionMotorPosition(double position) {
    rightExtensionMotor.getPIDController().setReference(convertPositionToPercent(position), ControlType.kSmartMotion);
  }

  public void setExtensionOutput(double percentOutput) {
    leftExtensionMotor.set(-percentOutput);
    rightExtensionMotor.set(percentOutput);
  }

  public void setFeedMotor(double percentOutput) {
    rotationMotor.set(percentOutput);
  }

  public void setInverted(boolean status){
    reverseMotor = status;
  }


  public static double convertPositionToPercent(double position) {
    double high = 3.6;
    double low = 0;
    return (position - low) / (high - low);

  }

  public double getIntakePosition() {
    return leftExtensionMotor.getEncoder().getPosition();
  }

  public double getFeedMotorVelocity() {
    return rotationMotor.getEncoder().getVelocity();
  }

  public double getFeedMotorCurrent() {
    return rotationMotor.getOutputCurrent();
  }

  public IntakeState getCurrentState() {
    double pos = convertPositionToPercent(Math.abs(leftExtensionMotor.getEncoder().getPosition()));
    if (pos < 0.3)
      return IntakeState.RETRACTED;
    return pos < 0.98 ? IntakeState.EXTENDED_CUBE : IntakeState.EXTENDED_CONE;
  }

  public boolean hasPiece(){
    return getFeedMotorCurrent() > OBJECT_CURRENT;
  }
}
