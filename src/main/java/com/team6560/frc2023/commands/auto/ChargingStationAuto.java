// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import com.kauailabs.navx.frc.AHRS;
import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargingStationAuto extends CommandBase {

  private final AHRS m_navx = new AHRS(Port.kMXP, (byte) 200);
  private final Drivetrain drivetrain;
  PIDController chargingStationPIDController;

  /** Creates a new ChargingStationAuto. */
  public ChargingStationAuto(Drivetrain drivetrain) {


    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    chargingStationPIDController = new PIDController(0, 0, 0);
    chargingStationPIDController.reset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.drive(
    //   ChassisSpeeds.fromFieldRelativeSpeeds(
    //     chargingStationPIDController.calculate(m_navx.getPitch(), 0),
    //     0,
    //     0,
    //     drivetrain.getGyroscopeRotation()
    //   )
    // );
    if (m_navx.getPitch() < 2.5) {
      drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          0.5,
          0,
          0,
          drivetrain.getGyroscopeRotation()
        )
      );
    } else if (m_navx.getPitch() > 2.5) {
      drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          -0.5,
          0,
          0,
          drivetrain.getGyroscopeRotation()
        )
      );
    } else {
      drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          0,
          0,
          0,
          drivetrain.getGyroscopeRotation()
        )
      );}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return !DriverStation.isAutonomousEnabled();
  }
}
