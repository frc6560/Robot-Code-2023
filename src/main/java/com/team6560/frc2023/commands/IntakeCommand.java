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
  private ArmCommand armCommand;
  private int rotationMotorFrames;
  private boolean initializeComplete;
  private int newThingFrames;
  private int newNewThingFrames;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, ArmCommand armCommand, Controls controls) {
    this.intake = intake;
    this.controls = controls;
    this.armCommand = armCommand;

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
    // if (rotationMotorFrames > 25) {
    //   intake.moveIntake(0.0);
    //   arm.setArmState(ArmPose.MEDIUM_CONE);
    //   newThingFrames++;
    //   if (newThingFrames > 15) {
    //     intake.moveIntake(-0.3);
    //     newNewThingFrames++;
    //     if (newNewThingFrames > 15) {
    //       intake.moveIntake(0.0);
    //       arm.setArmState(ArmPose.DEFAULT);
    //       initializeComplete = true;
    //     }
          
    //   }
    // } else {
    //   intake.moveIntake(0.3);
    //   rotationMotorFrames++;
    // }
      
    // if (!initializeComplete)
    //   return;
    // if (intake.getCurrentState() == IntakeState.EXTENDED)
    //   initializeComplete = true;
    
    
    // if (!initializeComplete)
    //   return;

    // intake.setIntakeState(controls.isIntakeDown() ? IntakeState.EXTENDED : IntakeState.RETRACTED);
    intake.moveIntake(controls.moveIntakeSpeed());
    // TODO: uncomment
    // intake.setRotationMotor(Math.abs(intake.getIntakePosition()) > 10.0 ? controls.intakeSpeed() : 0.0);
    // if () {

  //     armCommand.setArmStateLock(true);
  //     armCommand.setArmState(ArmPose.INTAKE_CONE);

  //     if (armCommand.transferFromIntake(Arm.armPoseMap.get(ArmPose.INTAKE_CONE).getClawSpeedMultiplier())) {
  //       intake.setRotationMotor(-controls.intakeSpeed());
  //       armCommand.transferFromIntake(0.0);
  //       armCommand.setArmStateLock(false);
  //     }

    // }
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
