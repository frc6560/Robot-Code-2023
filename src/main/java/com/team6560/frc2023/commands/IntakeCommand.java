// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Intake;
import com.team6560.frc2023.subsystems.Arm.ArmPose;
import com.team6560.frc2023.subsystems.Intake.IntakePose;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {
  private Intake intake;
  private Controls controls;
  private ArmCommand armCommand;
  private boolean cleared;

  private Timer intakeTimer = new Timer();
  private Timer initTimer = new Timer();
  private boolean initialized;
  private Timer initTimer2 = new Timer();

  public static interface Controls {
    IntakePose intakePose();

    boolean isCubeMode();
  }

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, ArmCommand armCommand, Controls controls) {
    this.intake = intake;
    this.controls = controls;
    this.armCommand = armCommand;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakePosition(1.0);
    initTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!initTimer.hasElapsed(0.5) && !initialized) {
      armCommand.setArmState(ArmPose.CLEARANCE);
      initTimer2.start();
    }
    if (!initTimer2.hasElapsed(0.5) && !initialized) {
      intake.setIntakeState(IntakePose.RETRACTED);
      initialized = true;
    }
    if (!initialized) {
      return;
    }
    
    IntakePose intakePose = controls.intakePose();
    if (intake.hasObject()) {
      intakePose = controls.isCubeMode() ? IntakePose.CUBE_TRANSFER : IntakePose.CONE_TRANSFER;
    }
    if (controls.intakePose() != IntakePose.RETRACTED && !cleared) {
      armCommand.setArmState(ArmPose.CLEARANCE);
      if (armCommand.isArmAtSetpoint()) {
        intake.setIntakeState(intakePose);
        this.cleared = true;
      }
    }
    if (controls.intakePose() !=IntakePose.RETRACTED && cleared) {
      armCommand.setArmState(intake.intakePoseMap.get(intakePose).getArmPose());
      intake.setIntakeState(intakePose);
    }

    if (armCommand.transferFromIntake(0.5)) {
      intake.setSuckMotor(0.5);
      intakeTimer.start();

    }

    if (intakeTimer.hasElapsed(0.5)) {
      intakeTimer.reset();
      armCommand.transferFromIntake(0.0);
      intake.setIntakeState(IntakePose.RETRACTED);
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
