package com.team6560.frc2023.commands;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.Constants.*;
import com.team6560.frc2023.Constants.ArmConstants.ArmPose;
import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.ArmState;
import com.team6560.frc2023.subsystems.GamePiece;
import com.team6560.frc2023.subsystems.Intake;
import com.team6560.frc2023.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {

  public interface Controls {
    boolean runIntake();

    boolean reverseIntake();

    boolean handOff();

    boolean isCubeMode();
  }

  private Intake intake;
  private Controls controls;
  private ArmCommand armCommand;
  
  private boolean initializing = true;
  private boolean handing = false;

  public IntakeCommand(Intake intake, ArmCommand armCommand, Controls controls) {
    this.intake = intake;
    this.controls = controls;
    this.armCommand = armCommand;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  private void init_sequence(){
    intake.setIntakeState(IntakeState.CLEARANCE);

    if(intake.isAtTargetState()) {
      armCommand.setArmState(IntakeConstants.ROTATION_ARM_CLEARANCE + 0.1);
    }

    if(armCommand.canRunIntake()){
      intake.setIntakeState(IntakeState.RETRACTED);
      
      if(intake.isAtTargetState()){
        initializing = false;
        armCommand.setArmState(ArmPose.DEFAULT);
      }
    }
  }

  private void handoff_sequence(boolean cubeMode){
    intake.setInverted(false);

    if(cubeMode){
      intake.setIntakeState(IntakeState.CLEARANCE);

      if(intake.isAtTargetState()){
        armCommand.setArmState(ArmPose.HUMAN_PLAYER_CUBE);
      }

      if(armCommand.canRunIntake()){
        intake.setIntakeState(IntakeState.RETRACTED);
      }

      if(intake.getCurrentState() == IntakeState.RETRACTED){
        armCommand.setArmState(ArmPose.LOW_CUBE);
      }


    } else{
      intake.setIntakeState(IntakeState.HANDOFF_CONE);
      armCommand.setArmState(ArmPose.INTAKE_CONE);

      if(armCommand.hasGamePiece()){
        intake.setIntakeState(IntakeState.RETRACTED);
        intake.setFeedMotor(IntakeConstants.HANDOFF_SPEED);
      }

      if(intake.getCurrentState() == IntakeState.RETRACTED){ // once fully done retracting
        armCommand.setArmState(ArmPose.DEFAULT);
        armCommand.setGroundIntakeMode(false);

        intake.setFeedMotor(0.0);
      }

    }
  }

  @Override
  public void execute() {
    if(initializing){
      init_sequence();
      return;
    }


    if(controls.runIntake()){
      armCommand.setGroundIntakeMode(true);

      boolean cubeMode = controls.isCubeMode();
      
      if(handing){
        handoff_sequence(cubeMode);

      } else if(cubeMode){
        intake.setIntakeState(IntakeState.EXTENDED_CUBE);
        armCommand.setArmState(ArmPose.INTAKE_CUBE);

        if(armCommand.hasGamePiece()){
          handing = true;
        }

      } else {
        intake.setIntakeState(IntakeState.EXTENDED_CONE);

        if(intake.hasPiece()){
          handing = true;
        }
      }
      
      intake.setInverted(controls.reverseIntake());

    } else {
      handing = false;

      armCommand.setArmState(ArmPose.HUMAN_PLAYER_CONE);

      if(intake.getCurrentState() != IntakeState.RETRACTED){
        if(armCommand.canRunIntake()){
          intake.setIntakeState(IntakeState.RETRACTED);

        } else{
          armCommand.setArmState(IntakeConstants.ROTATION_ARM_CLEARANCE + 0.1);
        }
      } else{
        armCommand.setArmState(ArmPose.DEFAULT);

        if(armCommand.isArmAtSetpoint()){
          armCommand.setGroundIntakeMode(false);
        }
      }
    }


  }

  @Override
  public void end(boolean interrupted) {
    intake.setFeedMotor(0);
    intake.setIntakeState(IntakeState.RETRACTED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
