// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.GamePiece;
import com.team6560.frc2023.subsystems.Arm.ArmPose;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmToPoseCommand extends CommandBase {
  private Arm arm;
  private ArmPose armPose;
  private boolean finished = false;
  private boolean place = true;

  private boolean startExtentionStatus;

  private Timer pistonTimer;

  private Timer clawTimer;
  private int clawSpeedSign;
  private GamePiece gamePiece;

  /** Creates a new MoveArmToPoseCommand. */
  public MoveArmToPoseCommand(Arm arm, ArmPose pose) {
    this.arm = arm;
    this.armPose = pose;
    addRequirements(arm);

    if (armPose == ArmPose.INTAKE_CONE || armPose == ArmPose.INTAKE_CUBE || armPose == ArmPose.GROUND_CONE || armPose == ArmPose.GROUND_CUBE) {
      this.clawSpeedSign = 1;
    } else {
      this.clawSpeedSign = -1;
    }

    if(armPose == ArmPose.LOW_CUBE || armPose == ArmPose.HIGH_CUBE || armPose == ArmPose.GROUND_CUBE || armPose == ArmPose.INTAKE_CUBE || armPose == ArmPose.MEDIUM_CUBE || armPose == ArmPose.HUMAN_PLAYER_CUBE) {
      this.gamePiece = GamePiece.CUBE;
    }

    if(armPose == ArmPose.LOW_CONE || armPose == ArmPose.HIGH_CONE || armPose == ArmPose.GROUND_CONE || armPose == ArmPose.INTAKE_CONE || armPose == ArmPose.MEDIUM_CONE || armPose == ArmPose.HUMAN_PLAYER_CONE || armPose == ArmPose.DOUBLE_SUB_CONE) {
      this.gamePiece = GamePiece.CONE;
    }

    // Use addRequirements() here to declare subsystem dependencies.
  }

  public MoveArmToPoseCommand(Arm arm, ArmPose pose, boolean place) {
    this(arm, pose);
    this.place = place;
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

    boolean isExtended = Arm.armPoseMap.get(armPose).getExtentionStatus();
    if (startExtentionStatus && !isExtended) {
      arm.setArmExtention(false);
      pistonTimer.start();

      if (!pistonTimer.hasElapsed(isExtended ? 1.0 : 0.5)) {
        return;
      } else {
        arm.setArmState(armPose);
      }
    }
    arm.setArmState(armPose);
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

        if ((gamePiece == GamePiece.CONE && pistonTimer.hasElapsed(isExtended ? 0.6 : 0.5)) || (gamePiece == GamePiece.CUBE && pistonTimer.hasElapsed(isExtended ? 0.3 : 0.2))) {
          double clawSpeed = Math.copySign(Arm.armPoseMap.get(armPose).getClawSpeedMultiplier(), clawSpeedSign);
          arm.setClawSpeed(clawSpeed);
          if (!place) {
            this.finished = true;
            return;
          }

          clawTimer.start();
        }

        if ((clawTimer.hasElapsed(clawSpeedSign > 0 ? 0.75 : 0.4))) {
          arm.setClawSpeed(0.0);
          this.finished = true;
        }

        if (!isExtended) {
          this.finished = true;
        }

      } else {
        if (!place) {
          this.finished = true;
          return;
        }
        arm.setClawSpeed(Math.copySign(Arm.armPoseMap.get(armPose).getClawSpeedMultiplier(), clawSpeedSign));
        clawTimer.start();

        if (clawTimer.hasElapsed(0.4)) {
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
