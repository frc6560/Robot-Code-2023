package com.team6560.frc2023.commands;

import com.team6560.frc2023.Constants;
import com.team6560.frc2023.Constants.*;
import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Arm.ArmPose;
import com.team6560.frc2023.subsystems.ArmState;
import com.team6560.frc2023.subsystems.GamePiece;
import com.team6560.frc2023.subsystems.Intake;
import com.team6560.frc2023.subsystems.Intake.IntakePose;
import com.team6560.frc2023.utility.NetworkTable.NtValueDisplay;
import com.team6560.frc2023.subsystems.IntakeState;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private boolean closing = true;

  private boolean handing = false;

  private boolean armGotObject = false;

  private NetworkTable nTable = NetworkTableInstance.getDefault().getTable("Intake");
  private NetworkTableEntry override;

  public IntakeCommand(Intake intake, ArmCommand armCommand, Controls controls) {
    this.intake = intake;
    this.controls = controls;
    this.armCommand = armCommand;

    addRequirements(intake);

    override = nTable.getEntry("Overide");
    override.setBoolean(false);
  }

  @Override
  public void initialize() {
    
    override.setBoolean(false);
    // initializing = true;
  }

  private void init_sequence(){
    armCommand.setArmStateLock(true);

    if(armCommand.canRunIntake()){
      intake.setIntakeState(IntakePose.RETRACTED);
      
      if(intake.atSetpoint() && intake.getCurrentPose() == IntakePose.RETRACTED){
        initializing = false;
        armCommand.setArmState(ArmPose.DEFAULT);
      }
    } else {
      intake.setIntakeState(IntakePose.CLEARANCE);
    }

    if(intake.getCurrentPose() == IntakePose.CLEARANCE && intake.atSetpoint()) {
      armCommand.setArmState(intake.intakePoseMap.get(IntakePose.CLEARANCE).getArmPose());
      
    }

  }

  private void closing_sequence(){

    handing = false;
    armGotObject = false;

    armCommand.setArmStateLock(true);
    armCommand.setArmState(intake.intakePoseMap.get(IntakePose.CLEARANCE).getArmPose());

    if(intake.getCurrentPose() != IntakePose.RETRACTED){
      if(armCommand.canRunIntake()){
        intake.setIntakeState(IntakePose.RETRACTED);

      } else{
        intake.setIntakeState(IntakePose.CLEARANCE);
        armCommand.setArmState(intake.intakePoseMap.get(IntakePose.CLEARANCE).getArmPose());
      }

    } else{
      armCommand.setArmState(ArmPose.CLEARANCE);
      armCommand.setArmStateLock(false);

      closing = false;
    }

  }

  private void handoff_sequence(boolean cubeMode){
    intake.setInverted(false);

    if(cubeMode){
      if(armCommand.canRunIntake()){
        intake.setIntakeState(IntakePose.RETRACTED);
      } else {
        intake.setIntakeState(IntakePose.CLEARANCE);
      }

      if(intake.getCurrentPose() == IntakePose.RETRACTED && intake.atSetpoint()){
        armCommand.setArmState(ArmPose.LOW_CUBE);

      } else if(intake.getCurrentPose() == IntakePose.CLEARANCE && intake.atSetpoint()){
        armCommand.setArmState(ArmPose.HUMAN_PLAYER_CUBE);
      }



    } else{
      if(!armGotObject){
        intake.setIntakeState(IntakePose.HANDOFF_CONE);
        armCommand.setClawSpeed(0.5);
      }

      System.out.println("has object " + armCommand.hasObject());

      if(armCommand.hasObject() || armGotObject){
        armGotObject = true;

        intake.setIntakeState(IntakePose.RETRACTED);
        intake.setSuckMotor(0.5);
      }

      if(intake.getCurrentPose() == IntakePose.RETRACTED && intake.atSetpoint()){ // once fully done retracting
        armCommand.setArmState(ArmPose.NONE);
        armCommand.setArmStateLock(false);
        armCommand.setClawSpeed(0.0);
        

        intake.setSuckMotor(0.0);
      }

    }
  }

  private void collapse_sequence(){
    armCommand.setArmStateLock(true);
    intake.setIntakeState(IntakePose.CLEARANCE);

    if(intake.getCurrentPose() == IntakePose.CLEARANCE && intake.atSetpoint()){
      armCommand.setArmState(ArmPose.DEFAULT);
    } 
    if(!armCommand.canRunIntake() && armCommand.isArmAtSetpoint()){
      intake.setIntakePosition(0.2);
    }
  }

  @Override
  public void execute() {
    if(override.getBoolean(false)){
      intake.setIntakePosition(1.05);
      armCommand.setArmStateLock(false);
      return;
    }

    if(initializing){
      init_sequence();
      return;
    }

    if(controls.runIntake()){

      closing = true;
      armCommand.setArmStateLock(true);

      boolean cubeMode = controls.isCubeMode();
      
      if(handing){
        handoff_sequence(cubeMode);

      } else if ((intake.getCurrentPose() == IntakePose.RETRACTED && intake.atSetpoint()) && !armCommand.canRunIntake()){
        armCommand.setArmState(intake.intakePoseMap.get(IntakePose.CLEARANCE).getArmPose());

      } else if(cubeMode){
        // if(armCommand.canRunIntake() || intake.getCurrentPose() == IntakePose.EXTENDED_CUBE)
          intake.setIntakeState(IntakePose.EXTENDED_CUBE);
        // else 
        //   armCommand.setArmState(ArmPose.CLEARANCE);

        if(intake.getCurrentPose() == IntakePose.EXTENDED_CUBE && intake.atSetpoint()){
          armCommand.setClawSpeed(0.5);
          armCommand.setArmState(ArmPose.INTAKE_CUBE);
          
          if(armCommand.isArmAtSetpoint()){
            intake.setSuckMotor(0.8);
          }
        }

        if(armCommand.hasObject()){
          handing = true;
          armCommand.setClawSpeed(0.05);
        }

      } else {
        intake.setIntakeState(IntakePose.EXTENDED_CONE);
        
        if(intake.getCurrentPose() == IntakePose.EXTENDED_CONE && intake.atSetpoint())
          armCommand.setArmState(ArmPose.INTAKE_CONE);

        if(intake.hasObject()){
          handing = true;
        }
      }
      
      intake.setInverted(controls.reverseIntake());

    } else if(closing){

      closing_sequence();
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

