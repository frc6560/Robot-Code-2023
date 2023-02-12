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

      double runClaw();
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
    rotationSpeed.setDouble(0.01);

    clawSpeed = ntTable.getEntry("Claw Speed (MOTOR RPM)");
    clawSpeed.setDouble(6560);
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
    if(controls.armRotation() != 0) System.out.println("rotating arm at " + controls.armRotation() * rotationSpeed.getDouble(0.0));

    arm.setArmRotationVelocity(controls.armRotation() * rotationSpeed.getDouble(0.0));

    if(controls.armExtention() && !prevControlArmExt){
      System.out.println("Setting extention piston " + (!arm.getExtentionStatus() ? "out." : "in."));

      arm.setArmExtention(!arm.getExtentionStatus());
    }

    arm.setClawSpeed(controls.runClaw() * clawSpeed.getDouble(0.0));

    if(controls.runClaw() != 0) System.out.println("Running claw at " + clawSpeed.getDouble(0.0));

    prevControlArmExt = controls.armExtention();
    prevControlBatteryExt = controls.armExtention();
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
