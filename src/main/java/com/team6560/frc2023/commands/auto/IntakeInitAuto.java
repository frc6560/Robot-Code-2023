// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Intake;
import com.team6560.frc2023.subsystems.Arm.ArmPose;
import com.team6560.frc2023.subsystems.Intake.IntakePose;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeInitAuto extends CommandBase {
  final Arm arm;
  final Intake intake;

  final boolean closing;
  final boolean cubeMode;

  boolean intakeCleared = false;
  boolean armCleared = false;
  boolean fin = false;


  /** Creates a new IntakeAuto. */
  public IntakeInitAuto(Intake intake, Arm arm, boolean cubeMode){
    this(intake, arm, cubeMode, false);
  }

  public IntakeInitAuto(Intake intake, Arm arm, boolean cubeMode, boolean closing) {
    this.intake = intake;
    this.arm = arm;
    this.closing = closing;
    this.cubeMode = cubeMode;

    addRequirements(intake, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fin = true;
    if(true) return; // new intake

    if(closing){
      close();
      return;

    } else {
      open();
      return;
    }

  }

  public void close(){
    arm.setArmState(ArmPose.CLEARANCE);

    if(arm.isArmAtSetpoint()){
      armCleared = true;
    }
    if(armCleared){
      intake.setIntakeState(IntakePose.RETRACTED);
      intake.setSuckMotor(0.0);
      fin = true;
    }

    // if(armCleared && (intake.getCurrentPose() == IntakePose.RETRACTED && intake.atSetpoint())){
    //   fin = true;
    // }
  }

  public void open(){
    intake.setIntakeState(IntakePose.CLEARANCE);
    arm.setClawSpeed(0.1);

    if(intake.getCurrentPose() == IntakePose.CLEARANCE && intake.atSetpoint()){
      intakeCleared = true;
      System.out.println("intake cleared");
    }
    if(intakeCleared){
      if(cubeMode)
        arm.setArmState(ArmPose.HIGH_CUBE);
      else 
        arm.setArmState(ArmPose.HIGH_CONE);
    }

    if(intakeCleared && arm.canRunIntake()){
      armCleared = true;
      System.out.println("arm cleared");
    }
    if(armCleared){
      intake.setIntakeState(IntakePose.RETRACTED);
      arm.setClawSpeed(0.0);
    }

    if(intakeCleared && armCleared && intake.atSetpoint()){
      fin = true;
      System.out.println("all cleared");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fin;
  }
}
