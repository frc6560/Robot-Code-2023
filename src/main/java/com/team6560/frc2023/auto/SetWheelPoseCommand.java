// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.auto;

import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetWheelPoseCommand extends CommandBase {

  private Drivetrain drivetrain;
  private boolean finish = false;

  private double fLdeg;
  private double fRdeg;
  private double bLdeg;
  private double bRdeg;

  public SetWheelPoseCommand(Drivetrain drivetrain, double fLdeg, double fRdeg, double bLdeg, double bRdeg) {
    this.drivetrain = drivetrain;

    this.fLdeg = fLdeg;
    this.fRdeg = fRdeg;
    this.bLdeg = bLdeg;
    this.bRdeg = bRdeg;
    
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setChassisState(fLdeg, fRdeg, bLdeg, bRdeg);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
