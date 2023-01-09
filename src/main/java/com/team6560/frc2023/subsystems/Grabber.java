// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.utility.Util;

public class Grabber extends SubsystemBase {

  private final CANSparkMax pivotMotor;
  private final CANSparkMax turretMotor;

  private static final double PIVOT_GEAR_RATIO = 1.0;
  private static final double TURRET_GEAR_RATIO = 1.0;

  private static final Rotation2d[] pivotMotorBounds = {Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(90.0)};
  private static final Rotation2d[] turretMotorBounds = {Rotation2d.fromDegrees(-10.0), Rotation2d.fromDegrees(10.0)};

  public Grabber() {
    pivotMotor = new CANSparkMax(Constants.pivotMotorId, MotorType.kBrushless);

    pivotMotor.getPIDController().setP(0);
    pivotMotor.getPIDController().setI(0);
    pivotMotor.getPIDController().setD(0);

    turretMotor = new CANSparkMax(Constants.turretMotorId, MotorType.kBrushless);

    turretMotor.getPIDController().setP(0);
    turretMotor.getPIDController().setI(0);
    turretMotor.getPIDController().setD(0);
  }

  public void setPivotRotation(Rotation2d rotation) {
    rotation = Util.getLimited(rotation, pivotMotorBounds[0], pivotMotorBounds[1]);
    pivotMotor.getPIDController().setReference(rotation.div(PIVOT_GEAR_RATIO).getRotations(), ControlType.kPosition);
  }

  public Rotation2d getPivotMotorRotation() {
    return Rotation2d.fromRotations(pivotMotor.getEncoder().getPosition());
  }

  public void setTurretRotation(Rotation2d rotation) {
    rotation = Util.getLimited(rotation, turretMotorBounds[0], turretMotorBounds[1]);
    turretMotor.getPIDController().setReference(rotation.div(TURRET_GEAR_RATIO).getRotations(), ControlType.kPosition);
  }

  public Rotation2d getTurretMotorRotation() {
    return Rotation2d.fromRotations(turretMotor.getEncoder().getPosition());
  }

  public void setGrabberRotation(Rotation3d rotation) {
    Rotation2d turretRotation = Rotation2d.fromRadians(rotation.getZ());
    Rotation2d pivotRotation = Rotation2d.fromRadians(rotation.getY());

    setTurretRotation(turretRotation);
    setPivotRotation(pivotRotation);
  }

  public Rotation3d getGrabberRotation() {
    return new Rotation3d(0.0, getPivotMotorRotation().unaryMinus().getRadians(), getTurretMotorRotation().getRadians());
  }

  public void setInterpolatedGrabberPosition(Pose3d pose) {
    Rotation3d rotation = pose.getRotation();
    setGrabberRotation(rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
