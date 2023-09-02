// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023;

// import java.io.File;

import com.team6560.frc2023.commands.DriveCommand;
import com.team6560.frc2023.controls.ManualControls;
import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        // not public or private so Robot.java has access to it.
        final Drivetrain drivetrain;
        private final DriveCommand driveCommand;

        private final ManualControls manualControls = new ManualControls(new XboxController(0), new XboxController(1));


        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                drivetrain = new Drivetrain();
                driveCommand = new DriveCommand(drivetrain, manualControls);

                drivetrain.setDefaultCommand(driveCommand);
        }


        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return null;
        }

}
