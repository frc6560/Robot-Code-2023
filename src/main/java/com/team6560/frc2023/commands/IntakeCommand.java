package com.team6560.frc2023.commands;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.Constants.*;
import com.team6560.frc2023.Constants.ArmConstants.ArmPose;
import com.team6560.frc2023.subsystems.Arm;
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

  private boolean handOff = false;
  private int handoffDebounce = 0;
  
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
      armCommand.setArmState(ArmPose.MEDIUM_CONE);

      if(armCommand.canRunIntake()){
        intake.setIntakeState(IntakeState.RETRACTED);
        initializing = false;
      }
    }
  }

  @Override
  public void execute() {
    if(initializing){
      init_sequence();
      return;
    }


    if(!handOff && controls.handOff()){ // incase operator accidentally clicks the handoff
      handoffDebounce++;

      if(handoffDebounce > 15) {
        handOff = true;
      }

    } else {
      handoffDebounce = 0;
      // handOff = false;
    }


    if(controls.runIntake()){
      armCommand.setGroundIntakeMode(true);

      boolean cubeMode = controls.isCubeMode();
      
      if(cubeMode){
        intake.setIntakeState(IntakeState.EXTENDED_CUBE);

        armCommand.setArmState(ArmPose.INTAKE_CUBE);

        if(armCommand.hasGamePiece()){
          handing = true;
        }

      } else {

        if(handOff){
          intake.setIntakeState(IntakeState.HANDOFF_CONE);
          if(armCommand.hasGamePiece()){ // Spit out the cone and go back
            handing = true;
          }

        } else{
          intake.setIntakeState(IntakeState.EXTENDED_CONE);
        }

        armCommand.setArmState(ArmPose.INTAKE_CONE);
      }

      if(armCommand.hasGamePiece()){
        armCommand.setArmState(IntakeConstants.ROTATION_ARM_CLEARANCE + 0.1);
      }

    } else {
      handOff = false;
      handoffDebounce = 0;

      armCommand.setArmState(ArmPose.HUMAN_PLAYER_CONE);

      if(armCommand.canRunIntake()){
        intake.setIntakeState(IntakeState.RETRACTED);
        armCommand.setGroundIntakeMode(false);
      }
    }

    intake.setInverted(controls.reverseIntake());

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
