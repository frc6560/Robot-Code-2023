// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import java.security.spec.AlgorithmParameterSpec;

import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Intake;
import com.team6560.frc2023.subsystems.Arm.ArmPose;
import com.team6560.frc2023.subsystems.Intake.IntakePose;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakePickupAuto extends CommandBase {
  final Intake intake;
  final Arm arm;
  final boolean cubeMode;

  boolean handing = false;

  boolean flag1 = false;
  boolean flag2 = false;
  boolean flag3 = false;

  boolean fin = false;
  
  public IntakePickupAuto(Intake intake, Arm arm, boolean cubeMode) {
    this.intake = intake;
    this.arm = arm;
    this.cubeMode = cubeMode;

    addRequirements(intake, arm);
  }


  @Override
  public void initialize() {
  }


  @Override
  public void execute() {
    if(cubeMode){
      runCube();
    } else {
      runCone();
    }
  }

  private void runCone(){
    if(handing) {
      handOffCone();
      return;
    }

    arm.setArmState(ArmPose.CLEARANCE);

    if(arm.isArmAtSetpoint()){
      flag1 = true;
    }
    if(flag1){
      intake.setIntakeState(IntakePose.EXTENDED_CONE);
    }

    if(flag1 && intake.atSetpoint()){
      flag2 = true;
    }
    if(flag2){
      arm.setArmState(ArmPose.INTAKE_CONE);
      arm.setClawSpeed(0.0);
    }

    if(flag1 && flag2 && arm.isArmAtSetpoint()){
      flag3 = true;
    }

    if(flag1 && flag2 && flag3 && intake.hasObject()){
      handing = true;
      
      flag1 = false;
      flag2 = false;
      flag3 = false;
    }
  }

  private void handOffCone(){
    arm.setArmState(ArmPose.INTAKE_CONE);
    arm.setClawSpeed(0.8);

    if(arm.isArmAtSetpoint()){
      flag1 = true;
    }
    if(flag1){
      intake.setIntakeState(IntakePose.HANDOFF_CONE);
    }

    if(flag1 && arm.hasCone()){
      flag2 = true;
    }
    if(flag2){
      intake.setIntakeState(IntakePose.RETRACTED);
      intake.setSuckMotor(0.8);

      arm.setClawSpeed(0.5);
    }

    if(flag1 && flag2 && intake.atSetpoint()){
      flag3 = true;
    }
    if(flag3){
      fin = true;
    }
  }


  private void runCube(){
    if(handing) {
      handOffCube();
      return;
    }

    if(!flag2) arm.setArmState(ArmPose.CLEARANCE);

    if(arm.isArmAtSetpoint()){
      flag1 = true;
    }
    if(flag1){
      intake.setIntakeState(IntakePose.CLEARANCE);
    }

    if(flag1 && intake.atSetpoint()){
      flag2 = true;
    }
    if(flag2){
      arm.setArmState(ArmPose.INTAKE_CUBE);
      arm.setClawSpeed(0.8);
    }

    if(flag1 && flag2 && arm.isArmAtSetpoint()){
      flag3 = true;
    }
    if(flag3){
      intake.setIntakeState(IntakePose.EXTENDED_CUBE);
      intake.setSuckMotor(0.9);
    }

    if(flag1 && flag2 && flag3 && arm.hasCube()){
      handing = true;
      
      flag1 = false;
      flag2 = false;
      flag3 = false;
    }

  }

  private void handOffCube(){
    if(!flag2) intake.setIntakeState(IntakePose.CLEARANCE);

    if(intake.atSetpoint()){
      flag1 = true;
    }
    if(flag1){
      arm.setArmState(ArmPose.HIGH_CUBE);
      arm.setClawSpeed(0.05);
    }

    if(flag1 && arm.isArmAtSetpoint()){
      flag2 = true;
    }
    if(flag2){
      arm.setClawSpeed(0.0);
      
      intake.setIntakeState(IntakePose.RETRACTED);
    }

    if(flag1 && flag2 && intake.atSetpoint()){
      fin = true;
    }

  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return fin;
  }
}
