// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;
import static com.team6560.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  CANSparkMax rightExtentionMotor = new CANSparkMax(CLIMB_RIGHT_EXTENTION_MOTOR, MotorType.kBrushless);
  CANSparkMax leftExtentionMotor = new CANSparkMax(CLIMB_LEFT_EXTENTION_MOTOR, MotorType.kBrushless);

  CANSparkMax driveMotor = new CANSparkMax(CLIMB_DRIVE_MOTOR, MotorType.kBrushless);
  
  public Climb() {
    rightExtentionMotor.restoreFactoryDefaults();
    leftExtentionMotor.restoreFactoryDefaults();

    driveMotor.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
