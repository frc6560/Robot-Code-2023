// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.GamePiece;
import com.team6560.frc2023.subsystems.Intake;
import com.team6560.frc2023.subsystems.Arm.ArmPose;
import com.team6560.frc2023.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {

  public interface Controls {
    boolean isIntakeDown();

    double moveIntakeSpeed();

    double intakeSpeed();
  }

  private Intake intake;
  private Controls controls;
  private Arm arm;
  private int rotationMotorFrames;
  private boolean initializeComplete;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, Arm arm, Controls controls) {
    this.intake = intake;
    this.controls = controls;
    this.arm = arm;

    initializeComplete = false;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (intake.getCurrentState() == IntakeState.EXTENDED)
    //   initializeComplete = true;
    
    
    // if (!initializeComplete)
    //   return;

    // intake.setIntakeState(controls.isIntakeDown() ? IntakeState.EXTENDED : IntakeState.RETRACTED);
    intake.moveIntake(controls.moveIntakeSpeed());
    if (intake.isSafeToRunRotationMotor())
      intake.setRotationMotor(controls.intakeSpeed());
    else if (intake.getCurrentGamePiece() == GamePiece.CONE) {

      if (arm.transferFromIntake()) {
        intake.setRotationMotor(0.4);
        rotationMotorFrames++;
      }

      if (rotationMotorFrames >= 20) {
        if (arm.transferFromIntakePart2()) {
          intake.setRotationMotor(0.0);
          rotationMotorFrames = 0;
        }
      }

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
