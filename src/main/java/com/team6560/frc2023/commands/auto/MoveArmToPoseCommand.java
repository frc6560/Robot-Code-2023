// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Arm.ArmPose;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmToPoseCommand extends CommandBase {
  private Arm arm;
  private ArmPose armPose;
  private boolean finished = false;
  private boolean startExtentionStatus;

  private Timer pistonTimer;

  private Timer clawTimer;

  /** Creates a new MoveArmToPoseCommand. */
  public MoveArmToPoseCommand(Arm arm, ArmPose pose) {
    this.arm = arm;
    this.armPose = pose;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startExtentionStatus = arm.getExtentionStatus();
    pistonTimer = new Timer();
    clawTimer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmState(armPose);
    
    boolean isExtended = Arm.armPoseMap.get(armPose).getExtentionStatus();
    // System.out.println(finished);

    if (arm.isArmAtSetpoint()) {

      if (this.armPose == ArmPose.DEFAULT) {
        arm.setArmExtention(false);
        this.finished = true;
        return;
      }

      if (isExtended != startExtentionStatus) {
        // System.out.println("THING!!");
        arm.setArmExtention(isExtended);
        pistonTimer.start();

        if (pistonTimer.hasElapsed(isExtended ? 1.5 : 1.0)) {
          double clawSpeed = Arm.armPoseMap.get(armPose).getClawSpeedMultiplier();
          arm.setClawSpeed(clawSpeed);
          clawTimer.start();
        }
        
        if (clawTimer.hasElapsed(1.5)) {
          arm.setClawSpeed(0.0);
          this.finished = true;
        }

        
      } else {
        arm.setClawSpeed(Arm.armPoseMap.get(armPose).getClawSpeedMultiplier());
        clawTimer.start();

        if (clawTimer.hasElapsed(1.5)) {
          arm.setClawSpeed(0.0);
          this.finished = true;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
