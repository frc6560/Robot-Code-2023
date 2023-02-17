// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.GamePiece;
import com.team6560.frc2023.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {

  public interface Controls {
    boolean isIntakeDown();

    double intakeSpeed();
  }

  private Intake intake;
  private Controls controls;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, Controls controls) {
    this.intake = intake;
    this.controls = controls;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeOut(controls.isIntakeDown());

    if (intake.isSafeToRunRotationMotor())
      intake.setRotationMotor(controls.intakeSpeed());
    else {

      if (intake.getCurrentGamePiece() == GamePiece.CONE)
        if ( (int) Timer.getFPGATimestamp() % 4 == 0) {
          intake.setRotationMotor(controls.intakeSpeed() / 4.0);
          return;
        }

      intake.setRotationMotor(0.0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
