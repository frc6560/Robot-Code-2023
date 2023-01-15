// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargingStationAuto extends CommandBase {

  private final Drivetrain drivetrain;
  private boolean finished = false;

  private Debouncer debouncer = new Debouncer(3.0, DebounceType.kBoth);

  /** Creates a new ChargingStationAuto. */
  public ChargingStationAuto(Drivetrain drivetrain) {

    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
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

    if (drivetrain.getCalculatedGyroPitchRoll().getDegrees() < -3.5) {
      finished = false;
      drivetrain.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              -0.3,
              0,
              0,
              drivetrain.getGyroscopeRotation()));
    } else if (drivetrain.getCalculatedGyroPitchRoll().getDegrees() > 3.5) {
      finished = false;
      drivetrain.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0.3,
              0,
              0,
              drivetrain.getGyroscopeRotation()));
    } else {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            0,
            drivetrain.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return debouncer.calculate(finished);
  }
}
