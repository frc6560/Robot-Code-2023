// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.Constants.IntakeConstants;
import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Arm.ArmPose;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that controls the arm subsystem.
 */
public class ArmCommand extends CommandBase {

  /**
   * Interface for defining the controls for the arm command.
   */
  public static interface Controls {
    ArmPose armState();
    double runClaw();
    boolean isCubeMode();
    boolean isOverridingArm();
    double armRotationOverride();
    boolean armExtentionOverride();
    boolean resetArmZero();
    boolean overrideArmSoftLimits();

    boolean isOverridingIntake();
  }

  private Arm arm;
  private Controls controls;
  private boolean prevControlArmExt = false;
  private NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");
  private NetworkTable ntTableIntake = NetworkTableInstance.getDefault().getTable("Intake");
  private NetworkTableEntry rotationSpeed;
  private NetworkTableEntry clawSpeed;
  private boolean lock;

  private NetworkTableEntry ignoreIntakeLock;
  
  private NetworkTableEntry ntArmEdgeWarning;

  private NetworkTableEntry ntSignalLight;

  private boolean toggleArmExtention = false;
  private boolean prevORExtention = false;
  private boolean hasPiece;


  /** 
   * Constructs a new ArmCommand object.
   *
   * @param arm the arm subsystem object
   * @param controls the controls object that defines how the arm should be controlled
   */
  public ArmCommand(Arm arm, Controls controls) {
    this.arm = arm;
    this.controls = controls;
    addRequirements(arm);

    rotationSpeed = ntTable.getEntry("Rotation Speed (ARM RPM)");
    rotationSpeed.setDouble(0.025);

    clawSpeed = ntTable.getEntry("Claw Speed (MOTOR RPM)");
    clawSpeed.setDouble(6560);
    
    ignoreIntakeLock = ntTable.getEntry("Ignore Intake?");
    ignoreIntakeLock.setBoolean(false);
    
    ntArmEdgeWarning = ntTable.getEntry("ARM EDGE WARNING");
    ntArmEdgeWarning.setBoolean(true);
    
    ntSignalLight = ntTableIntake.getEntry("Has game piece signal");
  }

  /** 
   * Initializes the command. Sets the arm's rotation velocity to 0.
   */
  @Override
  public void initialize() {
    arm.setArmRotationVelocity(0.0);
  }

  /**
   * Executes the command. 
   * 
   * Reads the control inputs and sets the arm's position and/or velocity based on them.
   */
  @Override
  public void execute() {
    if(controls.isOverridingIntake()){
      double edgeWarningThreshold = 0.05;

      arm.setClawSpeed(-0.1);
      arm.setBreakMotor(controls.armRotationOverride()/1.0);

      if(arm.getArmPose() < edgeWarningThreshold || (1-arm.getArmPose()) < edgeWarningThreshold){
        ntArmEdgeWarning.setBoolean(false);
      } else {
        ntArmEdgeWarning.setBoolean(true);
      }

      if(controls.armExtentionOverride() && !prevORExtention){
        toggleArmExtention = !toggleArmExtention;
      }

      arm.setArmExtention(toggleArmExtention);

      prevORExtention = controls.armExtentionOverride();

      return;

    } else {
      ntArmEdgeWarning.setBoolean(true);

      toggleArmExtention = false;
      prevORExtention = false;
    }

    if (lock && !ignoreIntakeLock.getBoolean(false)){
      return;
    }


    if (controls.resetArmZero()) {
      arm.resetArmZero();
    }

    double armSpeedMultiplyer;
    if (controls.armState() == ArmPose.NONE) { // If going manual mode
      armSpeedMultiplyer = controls.runClaw() > 0.0 ? (1.0) : 0.75;
      if (!controls.isCubeMode()) {
        armSpeedMultiplyer *= 0.4;
      }
    } else {
      armSpeedMultiplyer = Arm.armPoseMap.get(controls.armState()).getClawSpeedMultiplier();
    }

    if (controls.isCubeMode()) {
      if (controls.runClaw() > 0.0)
        armSpeedMultiplyer *= 0.85;
      else
        armSpeedMultiplyer *= 1.5;
    }
    
    this.setClawSpeed(armSpeedMultiplyer * controls.runClaw());

    if(arm.getClawSpeed() < 0.2){ // i have no idea why this works
      ntSignalLight.setBoolean(false);
    }

    if (controls.armExtentionOverride() && !prevControlArmExt) {
      arm.setArmExtention(!arm.getExtentionStatus());
    }

    prevControlArmExt = controls.armExtentionOverride();
    if (!controls.isOverridingArm() && controls.armState() != ArmPose.NONE) {
      double desiredState = Arm.armPoseMap.get(controls.armState()).getPosition();
      double currPose = arm.getArmPose();
      double thing = Math.abs(desiredState - currPose);
      System.out.println(thing);
      if (Math.abs(desiredState - currPose) > arm.convertRawArmPoseToArmPose(Arm.ALLOWED_ERROR)) {
        arm.setArmState(controls.armState());
        return;
      }
    }

    if (controls.overrideArmSoftLimits()) {
      arm.setArmRotationVelocityOverrideSoftLimits(controls.armRotationOverride() * rotationSpeed.getDouble(0.0));
    } else {
      arm.setArmRotationVelocity(controls.armRotationOverride() * rotationSpeed.getDouble(0.0));
    }
  }

  /**
   * Transfers a cube from the intake to the arm at the specified claw speed.
   *
   * @param clawSpeed the speed at which to transfer the cube
   * @return true if the transfer was successful, false otherwise
   */
  public boolean transferFromIntake(double clawSpeed) {
    return arm.transferFromIntake(clawSpeed);
  }
  
  public boolean hasObject(boolean cubeMode) {
    if(cubeMode)
      return arm.hasCube();
    return arm.hasCone();
    // return false;
  }

  public double getArmPosition(){
    return arm.getArmPose();
  }

  public void setArmState(ArmPose armPose) {
    arm.setArmState(armPose);
  }

  public void setArmState(double armPose) {
    arm.setArmState(armPose);
  }

  public void setClawSpeed(double output){// if(output != 0) System.out.println("Claw is running at " + output);
    if(Math.abs(output) <= 0.05){
      this.hasPiece = false;
    }
    if((controls.isCubeMode() && arm.hasCube()) || hasPiece){
      hasPiece = true;
      output = 0.05;
    }
    
    arm.setClawSpeed(output);
  }

  public boolean isArmAtSetpoint() {
    return arm.isArmAtSetpoint();
  }

  public void setArmStateLock(boolean lock) {
    this.lock = lock;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmExtention(false);
    arm.setArmRotationVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean canRunIntake(){
    return arm.canRunIntake();
  }
}
