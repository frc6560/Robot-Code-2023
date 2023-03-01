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

  private boolean handingOff = false;
  private int handoffDebounce = 0;
  
  private boolean initializing = true;

  public IntakeCommand(Intake intake, ArmCommand armCommand, Controls controls) {
    this.intake = intake;
    this.controls = controls;
    this.armCommand = armCommand;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(initializing){
      intake.setIntake(IntakeState.EXTENDED_CONE);

      if(intake.getCurrentState() != IntakeState.RETRACTED) {
        armCommand.setArmState(ArmPose.MEDIUM_CONE);

        if(armCommand.isArmAtSetpoint()){
          initializing = false;
          intake.setIntakeState(IntakeState.RETRACTED);
        }
      }
      
      return;
    }

    if(!handingOff && controls.handOff()){ // incase operator accidentally clicks the handoff
      handoffDebounce++;

      if(handoffDebounce > 15) {
        handingOff = true;
      }

    } else {
      handoffDebounce = 0;
    }

    if(controls.runIntake()){
      armCommand.setGroundIntakeMode(true);
      
      if(controls.isCubeMode()){
        intake.setIntakeState(IntakeState.EXTENDED_CUBE);

        armCommand.setArmState(ArmPose.INTAKE_CUBE);

      } else {
        if(handingOff){
          armCommand.setArmState(ArmPose.INTAKE_CONE);
          intake.setIntakeState(IntakeState.HANDOFF_CONE);

        } else{
          intake.setIntakeState(IntakeState.EXTENDED_CONE);
        }
      }

    } else {
      handingOff = false;
      handoffDebounce = 0;

      intake.setIntakeState(IntakeState.RETRACTED);
      armCommand.setGroundIntakeMode(false);
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
