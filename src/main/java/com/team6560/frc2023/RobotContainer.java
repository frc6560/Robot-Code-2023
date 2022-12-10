// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023;

import com.team6560.frc2023.commands.DriveCommand;
import com.team6560.frc2023.commands.auto.AutoBuilder;
import com.team6560.frc2023.controls.ManualControls;
import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final Drivetrain drivetrain = new Drivetrain();

        private final DriveCommand driveCommand;

        private final ManualControls manualControls = new ManualControls(new XboxController(0));

        // A chooser for autonomous commands
        private final SendableChooser<String> autoChooser = new SendableChooser<String>();

        private final AutoBuilder autoBuilder;


        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                driveCommand = new DriveCommand(drivetrain, manualControls);
                drivetrain.setDefaultCommand(driveCommand);

                autoBuilder = new AutoBuilder(drivetrain);

                // Add commands to the autonomous command chooser
                autoChooser.setDefaultOption("FullAuto", "FullAuto");

                autoChooser.addOption("New Path", "New Path");

                // Put the chooser on the dashboard
                Shuffleboard.getTab("Auto Choose").add(autoChooser);
        }


        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoBuilder.getAutoCommand(autoChooser.getSelected());
        }

}
