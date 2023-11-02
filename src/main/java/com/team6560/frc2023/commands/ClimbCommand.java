// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbCommand extends CommandBase {










  private Controls controls;

  private Climb climb;

  public static interface Controls {
    boolean putClimbUp();
    boolean putClimbDown();
  }

  /** Creates a new ClimbCommand. */
  public ClimbCommand(Climb climb, Controls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.putClimbUp()) {
      climb.setPos(0.0);
    }
    if (controls.putClimbDown()) {
      climb.setPos(0.0);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
