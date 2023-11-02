// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team6560.frc2023.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;


public class Climb extends SubsystemBase {



  WPI_TalonFX ClimbMotorLeft;
  WPI_TalonFX ClimbMotorRight;

  NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("CLimb");







  double maxClimbRotation;
  double minClimbRotation;

  /** Creates a new Climb. */
  public Climb() {
    maxClimbRotation = 0.0;
    minClimbRotation = 0.0;

    ClimbMotorLeft = new WPI_TalonFX(Constants.LEFT_CLIMB_MOTOR);
    ClimbMotorRight = new WPI_TalonFX(Constants.RIGHT_CLIMB_MOTOR);

    // isUsingVelocity = false;

    ClimbMotorLeft.configFactoryDefault();
    ClimbMotorRight.configFactoryDefault();

    ClimbMotorLeft.setSelectedSensorPosition(0);

    ClimbMotorLeft.config_kP(0, 0.1); // Dont need these, make it manual control not auto
    ClimbMotorLeft.config_kI(0, 0.0); //      so you only need open loop speed control
    ClimbMotorLeft.config_kD(0, 0.0);
    ClimbMotorLeft.config_kF(0, 0);

    ClimbMotorLeft.setInverted(true);

    ClimbMotorRight.follow(ClimbMotorLeft);
    ClimbMotorRight.setInverted(InvertType.OpposeMaster);

    // Set climb motors neutral mode to kBreak

    // ClimbMotorLeft.configForwardSoftLimitThreshold(maxClimbRotation);  // try using these for soft limits
    // ClimbMotorLeft.configReverseSoftLimitThreshold(minClimbRotation);


    ntDispTab("Climb")
        .add("Horizontal Position", () -> getClimbPose());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getClimbPose() {
    return ClimbMotorLeft.getSelectedSensorPosition() / Constants.TALONFX_POS_TO_ROTATION;
  }



  public void setPos(double targetPosRotation) { // Instead of position control use velocity control

    if (targetPosRotation > maxClimbRotation) {  // Try using the build in soft limits (line 66)
      targetPosRotation = maxClimbRotation;
    } else if (targetPosRotation < minClimbRotation) {
      targetPosRotation = minClimbRotation;
    }

    ClimbMotorLeft.set(TalonFXControlMode.Position, targetPosRotation * Constants.TALONFX_POS_TO_ROTATION);
    // VerticalMotorRight.set(TalonFXControlMode.Position, targetPosRotation);
  }


}
