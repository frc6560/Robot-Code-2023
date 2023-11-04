// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Climb;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbCommand extends CommandBase {



  private Controls controls;

  private Climb climb;

  NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Climb 2");
  NetworkTableEntry climbSpeed;

  final double DEFAULT_CLIMB_SPEED = 0.2;

  public static interface Controls {
    double getClimb();
  }

  /** Creates a new ClimbCommand. */
  public ClimbCommand(Climb climb, Controls controls) {
    this.climb = climb;
    this.controls = controls;

    addRequirements(climb);

    climbSpeed = ntTable.getEntry("Climb Speed");
    climbSpeed.setDouble(DEFAULT_CLIMB_SPEED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.runClimb(controls.getClimb() * climbSpeed.getDouble(0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.runClimb(0);
    // set climb velocity to 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
