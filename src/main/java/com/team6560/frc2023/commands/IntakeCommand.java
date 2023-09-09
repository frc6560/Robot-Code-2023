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

    double overideIntake();

    boolean isOverridingIntake();
  }

  private Intake intake;
  private Controls controls;
  private ArmCommand armCommand;
  
  private boolean initializing = true;
  private boolean closing = true;

  private boolean flag1 = false;
  private boolean flag2 = false;
  private boolean flag3 = false;

  private boolean handing = false;

  private boolean armGotObject = false;

  private NetworkTable nTable = NetworkTableInstance.getDefault().getTable("Intake");
  private NetworkTableEntry ntOverideToggle;
  private NetworkTableEntry ntIntakeEdgeWarning;

  private NetworkTableEntry ntIgnoreIntake = NetworkTableInstance.getDefault().getTable("Arm").getEntry("Ignore Intake?");

  private NetworkTableEntry ntSignalLight;

  private final Timer coneTimer = new Timer();

  public IntakeCommand(Intake intake, ArmCommand armCommand, Controls controls) {
    this.intake = intake;
    this.controls = controls;
    this.armCommand = armCommand;

    addRequirements(intake);

    ntOverideToggle = nTable.getEntry("OVERRIDE INTAKE");
    ntOverideToggle.setBoolean(false);

    ntIntakeEdgeWarning = nTable.getEntry("INTAKE EDGE WARNING");
    ntIntakeEdgeWarning.setBoolean(true);

    ntSignalLight = nTable.getEntry("Has game piece signal");
    ntSignalLight.setBoolean(false);
  }

  @Override
  public void initialize() {
    if(Math.abs(intake.getIntakePosition() ) < IntakeConstants.INTAKE_ACCEPTABLE_ERROR && Math.abs(armCommand.getArmPosition()) < Arm.ALLOWED_ERROR){
      initializing = true;
    } else {
      initializing = false;
    }
  }

  @Override
  public void execute() {

    ntOverideToggle.setBoolean(controls.isOverridingIntake());
    if(controls.isOverridingIntake()){
      double edgeWarningThreshold = 0.05;

      intake.setIntakeOveride(controls.overideIntake()/5);
      intake.setSuckMotor(0.1);

      if(intake.getIntakePosition() < edgeWarningThreshold || (1-intake.getIntakePosition()) < edgeWarningThreshold){
        ntIntakeEdgeWarning.setBoolean(false);
      } else {
        ntIntakeEdgeWarning.setBoolean(true);
      }

      closing = true;
      return;
    } else {
      ntIntakeEdgeWarning.setBoolean(true);
    }

    if(ntIgnoreIntake.getBoolean(false)){
      if(armCommand.canRunIntake())
        intake.setIntakeState(IntakePose.RETRACTED);

      intake.setSuckMotor(0.0);

      return;
    }

    if(initializing){
      // initializing = false;
      init_sequence();
      return;
    }
    boolean cubeMode = controls.isCubeMode();

    if(controls.runIntake()){
      armCommand.setArmStateLock(true);

      if(cubeMode){
        runCube();
      } else {
        runCone();
      }

      closing = true;

    } else if(closing){
      closing_sequence(cubeMode);
    } else {
      
      flag1 = false;
      flag2 = false;
      flag3 = false;
    }
  }


  private void init_sequence(){
    flag1 = false;
    flag2 = false;
    flag3 = false;

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

  private void closing_sequence(boolean cubeMode){
    boolean fin = false;
    
    flag1 = false;
    flag2 = false;
    flag3 = false;

    handing = false;

    armCommand.setArmStateLock(true);
    
    armCommand.setClawSpeed(0.05);

    if(intake.getCurrentPose() == IntakePose.RETRACTED && intake.atSetpoint()){
      fin = true;
    }

    if(cubeMode){
      if(!armCommand.canRunIntake()){
        intake.setIntakeState(IntakePose.CLEARANCE);

        if(intake.atSetpoint()) {
          armCommand.setArmState(ArmPose.CLEARANCE);
        }

      } else {
        intake.setIntakeState(IntakePose.RETRACTED);
      }
    } else {
      armCommand.setArmState(ArmPose.CLEARANCE);

      if(armCommand.canRunIntake()){
        intake.setIntakeState(IntakePose.RETRACTED);
        intake.setSuckMotor(0.7);
      }

    }

    if(fin){
      armCommand.setArmStateLock(false);
      armCommand.setClawSpeed(0.0);
      intake.setSuckMotor(0.0);

      closing = false;
    }

  }

  
  private void runCone(){
    ntSignalLight.setBoolean(false);
    
    if(handing) {
      handOffCone();
      return;
    }

    if(!flag2) {
      armCommand.setArmState(ArmPose.CLEARANCE);
    }

    if(armCommand.isArmAtSetpoint()){
      flag1 = true;
    }
    if(flag1){
      intake.setIntakeState(IntakePose.EXTENDED_CONE);
    }

    if(flag1 && intake.atSetpoint()){
      flag2 = true;
    }
    if(flag2){
      armCommand.setArmState(ArmPose.INTAKE_CONE);
      armCommand.setClawSpeed(0.0);
    }

    if(flag1 && flag2 && armCommand.isArmAtSetpoint()){
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
    ntSignalLight.setBoolean(true);

    armCommand.setArmState(ArmPose.INTAKE_CONE);
    armCommand.setClawSpeed(0.8);

    if(armCommand.isArmAtSetpoint()){
      flag1 = true;
    }
    if(flag1){
      intake.setIntakeState(IntakePose.HANDOFF_CONE);
    }

    if(flag1 && intake.atSetpoint() && armCommand.hasObject(false)){
      flag2 = true;
    }
    if(flag2){
      intake.setIntakeState(IntakePose.RETRACTED);
      intake.setSuckMotor(0.9);

      armCommand.setClawSpeed(0.5);
    }

    if(flag1 && flag2 && intake.atSetpoint()){
      flag3 = true;
    }
    if(flag3){
      intake.setSuckMotor(0.0);
      armCommand.setClawSpeed(0.0);

      armCommand.setArmStateLock(false);
    }
  }


  private void runCube(){
    ntSignalLight.setBoolean(false);
    
    if(handing) {
      handOffCube();
      return;
    }

    if(!flag2) armCommand.setArmState(ArmPose.CLEARANCE);

    if(armCommand.isArmAtSetpoint()){
      flag1 = true;
    }
    if(flag1){
      intake.setIntakeState(IntakePose.CLEARANCE);
      intake.setSuckMotor(0.8);
    }

    if(flag1 && intake.atSetpoint()){
      flag2 = true;
    }
    if(flag2){
      armCommand.setArmState(ArmPose.INTAKE_CUBE);
      armCommand.setClawSpeed(0.8);

      // armCommand.setArmState(0.0);
      // armCommand.setClawSpeed(0.0);
    }

    if(flag1 && flag2 && armCommand.isArmAtSetpoint()){
      flag3 = true;
    }
    if(flag3){
      intake.setIntakeState(IntakePose.EXTENDED_CUBE);
      intake.setSuckMotor(0.8);
    }

    if(flag1 && flag2 && flag3 && armCommand.hasObject(true)){
      handing = true;
      
      flag1 = false;
      flag2 = false;
      flag3 = false;
    }

  }

  private void handOffCube(){
    ntSignalLight.setBoolean(true);
    
    if(!flag2) intake.setIntakeState(IntakePose.CLEARANCE);

    if(intake.atSetpoint()){
      flag1 = true;
    }
    if(flag1){
      armCommand.setArmState(ArmPose.CLEARANCE);
      armCommand.setClawSpeed(0.05);
    }

    if(flag1 && armCommand.isArmAtSetpoint()){
      flag2 = true;
    }
    if(flag2){
      armCommand.setClawSpeed(0.0);
      
      intake.setIntakeState(IntakePose.RETRACTED);
    }

    if(flag1 && flag2 && intake.atSetpoint()){
      armCommand.setArmStateLock(true);
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

