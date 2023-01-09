// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GoToDoubleSubstationCommand extends CommandBase {
  
  private AutoBuilder autoBuilder;

  private Command pathFollow;

  public GoToDoubleSubstationCommand(AutoBuilder autoBuilder) {
    this.autoBuilder = autoBuilder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathFollow = autoBuilder.goToPose(new Pose2d()); // TODO: change desired pose
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pathFollow != null)
      pathFollow.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathFollow == null) return false;
    return pathFollow.isFinished();
  }
}
