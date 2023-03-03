// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.ArmState;
import com.team6560.frc2023.Constants;
import com.team6560.frc2023.Constants.*;
import com.team6560.frc2023.Constants.ArmConstants.ArmPose;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmCommand extends CommandBase {

  /**
   * Interface for defining the controls for the arm command.
   */
  public static interface Controls {
    Constants.ArmConstants.ArmPose armState();

    double runClaw();

    boolean isCubeMode();

    boolean isOverridingArm();

    double armRotationOverride();

    boolean armExtentionOverride();

    boolean resetArmZero();

    boolean overrideArmSoftLimits();
  }

  private Arm arm;
  private Controls controls;

  private boolean prevControlArmExt = false;

  private NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");
  private NetworkTableEntry rotationSpeed;
  private NetworkTableEntry clawSpeed;

  private Constants.ArmConstants.ArmPose targetState;
  
  private boolean groundIntake;

  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm, Controls controls) {
    this.arm = arm;
    this.controls = controls;

    addRequirements(arm);

    rotationSpeed = ntTable.getEntry("Rotation Speed (ARM RPM)");
    rotationSpeed.setDouble(0.015);

    clawSpeed = ntTable.getEntry("Claw Speed (MOTOR RPM)");
    clawSpeed.setDouble(6560);

    targetState = Constants.ArmConstants.ArmPose.NONE;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmRotationVelocity(0.0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.resetArmZero()) {
      arm.resetArmZero();
    }
    
    if(groundIntake){
      arm.setClawSpeed(Constants.ArmConstants.armPoseMap.get(targetState).getClawSpeedMultiplier());

      return;
    }


    double armSpeedMultiplyer;
    if (controls.armState() == Constants.ArmConstants.ArmPose.NONE) { // If going manual mode
      armSpeedMultiplyer = controls.runClaw() > 0.0 ? 1.0 : 0.25;
    } else
      armSpeedMultiplyer = Constants.ArmConstants.armPoseMap.get(controls.armState()).getClawSpeedMultiplier();

    if (controls.isCubeMode()) {
      if (controls.runClaw() > 0.0)
        armSpeedMultiplyer *= 0.85;
      else
        armSpeedMultiplyer *= 1.5;
    }
    arm.setClawSpeed(armSpeedMultiplyer * controls.runClaw());

    

    // if(controls.runClaw() != 0) System.out.println("Running claw at " +
    // clawSpeed.getDouble(0.0));

    if (controls.armExtentionOverride() && !prevControlArmExt) {
      // System.out.println("Setting extention piston " + (!arm.getExtentionStatus() ?
      // "out." : "in."));

      arm.setArmExtention(!arm.getExtentionStatus());
    }

    prevControlArmExt = controls.armExtentionOverride();
    if (!controls.isOverridingArm() && controls.armState() != Constants.ArmConstants.ArmPose.NONE  && !groundIntake) {
      // arm.setArmExtention(!arm.getExtentionStatus());

      arm.setArmState(controls.armState());

      return;
    }

    // if(controls.armRotationOverride() != 0) System.out.println("rotating arm at "
    // + controls.armRotationOverride() * rotationSpeed.getDouble(0.0));

    if (controls.overrideArmSoftLimits()) {
      arm.setArmRotationVelocityOverrideSoftLimits(controls.armRotationOverride() * rotationSpeed.getDouble(0.0));

    } else {
      arm.setArmRotationVelocity(controls.armRotationOverride() * rotationSpeed.getDouble(0.0));

    }

  }

  public boolean transferFromIntake(double clawSpeed) {
    return arm.transferFromIntake(clawSpeed);
  }

  public void setArmState(Constants.ArmConstants.ArmPose armPose) {
    targetState = armPose;
    arm.setArmState(armPose);
  }

  public boolean canRunIntake(){
    return arm.getArmPose() > IntakeConstants.ROTATION_ARM_CLEARANCE;
  }

  public boolean hasGamePiece(){
    return arm.getClawCurrentOutput() > 25;
  }
  
  public void setGroundIntakeMode(boolean status){
    this.groundIntake = status;
  }

  public boolean isArmAtSetpoint(){
    return arm.isArmAtSetpoint();
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
}
