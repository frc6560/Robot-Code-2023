// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargingStationAuto extends CommandBase {

  private final Drivetrain drivetrain;
  private double pitchOffsetDegrees;
  private double rollOffsetDegrees;
  private static final double k = 0.035;

  /** Creates a new ChargingStationAuto. */
  public ChargingStationAuto(Drivetrain drivetrain, double pitchOffsetDegrees, double rollOffsetDegrees) {

    addRequirements(drivetrain);
    this.drivetrain = drivetrain;

    this.pitchOffsetDegrees = pitchOffsetDegrees;
    this.rollOffsetDegrees = rollOffsetDegrees;
  }

  public ChargingStationAuto(Drivetrain drivetrain) {
    this(drivetrain, drivetrain.getPitch().getDegrees(), drivetrain.getRoll().getDegrees());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.drive(
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    // chargingStationPIDController.calculate(drivetrain.getCalculatedGyroPitchRoll().getDegrees(),
    // 0),
    // 0,
    // 0,
    // drivetrain.getGyroscopeRotation()
    // )
    // );

    // if (drivetrain.getCalculatedGyroPitchRoll().getDegrees() < -3.5) {
    //   drivetrain.drive(
    //       ChassisSpeeds.fromFieldRelativeSpeeds(
    //           0.225,
    //           0,
    //           0,
    //           drivetrain.getGyroscopeRotation()));
    // } else if (drivetrain.getCalculatedGyroPitchRoll().getDegrees() > 3.5) {
    //   drivetrain.drive(
    //       ChassisSpeeds.fromFieldRelativeSpeeds(
    //           -0.225,
    //           0,
    //           0,
    //           drivetrain.getGyroscopeRotation()));
    // } else {
    //   drivetrain.drive(
    //       ChassisSpeeds.fromFieldRelativeSpeeds(
    //           0,
    //           0,
    //           0,
    //           drivetrain.getGyroscopeRotation()));
    // }
    double speed_x = 0.0;
    double speed_y = 0.0;
    if (Math.hypot(drivetrain.getPitch().getDegrees() - pitchOffsetDegrees, drivetrain.getRoll().getDegrees() - rollOffsetDegrees) > 1.5) {
      speed_x = (drivetrain.getRoll().getDegrees() - rollOffsetDegrees)* k;
      speed_y = (drivetrain.getPitch().getDegrees() - pitchOffsetDegrees)* k;
    }

    drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speed_x,
            speed_y,
            0,
            drivetrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return !DriverStation.isAutonomousEnabled();
  }
}
