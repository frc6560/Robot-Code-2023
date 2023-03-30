// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands;


import com.team6560.frc2023.subsystems.LightItUpUpUpLightItUpUpUp;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LightItUpUpUpLightItUpUpUpCommand extends CommandBase {
  public interface Controls {
    boolean isCubeMode();
  }

  private LightItUpUpUpLightItUpUpUp subsystem;
  private Controls controls;

  private final Timer blinkTimer = new Timer();

  private NetworkTable nTable = NetworkTableInstance.getDefault().getTable("Intake");
  private NetworkTableEntry ntSignalLight;
  

  /** Creates a new LightItUpUpUpLightItUpUpUpCommand. */
  public LightItUpUpUpLightItUpUpUpCommand(LightItUpUpUpLightItUpUpUp subsystem, Controls controls) {
    this.subsystem = subsystem;
    this.controls = controls;
    addRequirements(subsystem);

    ntSignalLight = nTable.getEntry("Has game piece signal");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ntSignalLight.getBoolean(false)) {
      blinkTimer.start();
      // if (((int) (blinkTimer.get())) % 2 == 0) {
        subsystem.setColor(Color.kRed);
      // } else {
      //   subsystem.setColor(controls.isCubeMode() ? Color.kPurple : Color.kYellow);
      // }

      if (blinkTimer.hasElapsed(4)) {
        // ntSignalLight.setBoolean(false);
        blinkTimer.reset();
      }
    } else {
      subsystem.setColor(controls.isCubeMode());

    }
      

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
