package com.team6560.frc2023.commands;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.Constants.*;
import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Arm.ArmPose;
import com.team6560.frc2023.subsystems.ArmState;
import com.team6560.frc2023.subsystems.GamePiece;
import com.team6560.frc2023.subsystems.Intake;
import com.team6560.frc2023.subsystems.Intake.IntakePose;
import com.team6560.frc2023.subsystems.IntakeState;

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
    intake.setIntakeState(IntakePose.EXTENDED_CONE);

    if(intake.atSetpoint()) {
      armCommand.setArmState(IntakeConstants.ROTATION_ARM_CLEARANCE + 0.1);
    }

    if(armCommand.canRunIntake()){
      intake.setIntakeState(IntakePose.RETRACTED);
      
      if(intake.atSetpoint()){
        initializing = false;
        armCommand.setArmState(ArmPose.DEFAULT);
      }
    }
  }

  private void handoff_sequence(boolean cubeMode){

    if(cubeMode){
      intake.setIntakeState(IntakePose.EXTENDED_CUBE);

      if(intake.atSetpoint()){
        armCommand.setArmState(ArmPose.HUMAN_PLAYER_CUBE);
      }

      if(armCommand.canRunIntake()){
        intake.setIntakeState(IntakePose.RETRACTED);
      }

      if(intake.getCurrentPose() == IntakePose.RETRACTED){
        armCommand.setArmState(ArmPose.LOW_CUBE);
      }


    } else{
      intake.setIntakeState(IntakePose.HANDOFF_CONE);
      armCommand.setArmState(ArmPose.INTAKE_CONE);

      if(armCommand.transferFromIntake(0.5)){
        intake.setIntakeState(IntakePose.RETRACTED);
        intake.setSuckMotor(IntakeConstants.HANDOFF_SPEED);
      }

      if(intake.getCurrentPose() == IntakePose.RETRACTED){ // once fully done retracting
        armCommand.setArmState(ArmPose.DEFAULT);
        armCommand.setArmStateLock(false);
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
      armCommand.setArmStateLock(true);

      boolean cubeMode = controls.isCubeMode();
      
      if(handing){
        handoff_sequence(cubeMode);

      } else if (intake.getCurrentPose() == IntakePose.RETRACTED && !armCommand.canRunIntake()){
        armCommand.setArmState(IntakeConstants.ROTATION_ARM_CLEARANCE + 0.1);

      } else if(cubeMode){
        intake.setIntakeState(IntakePose.EXTENDED_CUBE);
        armCommand.setArmState(ArmPose.INTAKE_CUBE);

        if(armCommand.transferFromIntake(0.5)){
          handing = true;
        }

      } else {
        intake.setIntakeState(IntakePose.EXTENDED_CONE);

        if(intake.hasObject()){
          handing = true;
        }
      }
      
      intake.setInverted(controls.reverseIntake());

    } else {
      handing = false;

      armCommand.setArmState(ArmPose.HUMAN_PLAYER_CONE);

      if(intake.getCurrentPose() != IntakePose.RETRACTED){
        if(armCommand.canRunIntake()){
          intake.setIntakeState(IntakePose.RETRACTED);

        } else{
          intake.setIntakeState(IntakePose.EXTENDED_CONE);
          armCommand.setArmState(IntakeConstants.ROTATION_ARM_CLEARANCE + 0.1);
        }
      } else{
        armCommand.setArmState(ArmPose.DEFAULT);

        if(armCommand.isArmAtSetpoint()){
          armCommand.setArmStateLock(false);
        }
      }
    }


  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeState(IntakePose.RETRACTED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}