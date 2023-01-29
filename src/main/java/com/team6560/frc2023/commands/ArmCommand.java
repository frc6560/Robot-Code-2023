// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;

import com.team6560.frc2023.subsystems.Arm;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmCommand extends CommandBase {
  
    /**
     * Interface for defining the controls for the arm command.
     */
    public static interface Controls {
      double armRotation();

      boolean armExtention();

      boolean runClaw();

      boolean pullBattery();
  }

  Arm arm;
  Controls controls;

  boolean prevControlArmExt = false;
  boolean prevControlBatteryExt = false;

  NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");
  NetworkTableEntry rotationSpeed;
  NetworkTableEntry clawSpeed;
  /** Creates a new ArmCommand. */
  public ArmCommand(Arm arm, Controls controls) {
    this.arm = arm;
    this.controls = controls;

    addRequirements(arm);
    
    rotationSpeed = ntTable.getEntry("Rotation Speed (ARM RPM)");
    rotationSpeed.setDouble(0.5);

    clawSpeed = ntTable.getEntry("Claw Speed (MOTOR RPM)");
    clawSpeed.setDouble(6560);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmRotation(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmRotation(controls.armRotation() * rotationSpeed.getDouble(0.0));

    if(controls.armExtention() && !prevControlArmExt){
      arm.setArmExtention(!arm.getExtentionStatus());
    }

    if(controls.pullBattery() && !prevControlBatteryExt){
      arm.setBatteryExtention(!arm.getBatteryStatus());
    }

    arm.setGripperRollers(controls.runClaw() ? clawSpeed.getDouble(0.0) : 0.0);

    prevControlArmExt = controls.armExtention();
    prevControlBatteryExt = controls.armExtention();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmExtention(false);
    arm.setArmRotation(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
