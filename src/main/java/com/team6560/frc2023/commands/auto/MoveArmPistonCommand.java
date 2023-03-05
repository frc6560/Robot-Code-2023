// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import com.team6560.frc2023.subsystems.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmPistonCommand extends CommandBase {
  private Timer pistonTimer;
  private Arm arm;
  private boolean isExtended;

  /** Creates a new MoveArmPistonCommand. */
  public MoveArmPistonCommand(Arm arm, boolean isExtended) {
    this.arm = arm;
    this.isExtended = isExtended;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pistonTimer = new Timer();
    pistonTimer.start();
    arm.setArmExtention(isExtended);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pistonTimer.hasElapsed(0.5);
  }
}
