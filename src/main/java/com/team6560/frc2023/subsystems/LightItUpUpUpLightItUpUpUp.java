// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.team6560.frc2023.Constants;
import com.team6560.frc2023.controls.ManualControls;

public class LightItUpUpUpLightItUpUpUp extends SubsystemBase {
  private final CANdle candle;
  private boolean isCube;

  public LightItUpUpUpLightItUpUpUp() {
    this.candle = new CANdle(Constants.CANdleId);
  }

  public boolean isCANdleInCubeMode() {
    return isCube;
  }

  public void setColor(boolean isCube){
    this.isCube = isCube;
    if (isCube) 
      candle.setLEDs(171, 32, 253);
    else
      candle.setLEDs(255, 255, 0);
  }

  @Override
  public void periodic() {
  }
  
}
