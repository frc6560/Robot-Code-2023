// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023;

import com.team6560.frc2023.commands.ArmCommand;

// import java.io.File;

import com.team6560.frc2023.commands.DriveCommand;
import com.team6560.frc2023.commands.auto.AutoBuilder;
import com.team6560.frc2023.controls.ManualControls;
import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Drivetrain;
import com.team6560.frc2023.subsystems.Limelight;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PrintCommand;

public class RobotContainer {
        // The robot's subsystems and commands are defined here...

        // not public or private so Robot.java has access to it.
        Drivetrain drivetrain = null;

        private final Limelight limelight;

        private final DriveCommand driveCommand;

        private final Arm arm;

        private final ManualControls manualControls = new ManualControls(new XboxController(0));//, new XboxController(1));

        // A chooser for autonomous commands
        private final SendableChooser<String> autoChooser;

        private final AutoBuilder autoBuilder;


        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                limelight = new Limelight(manualControls, () -> drivetrain == null ? null : drivetrain.getPose());

                drivetrain = new Drivetrain(() -> limelight.getBotPose());
                
                arm = new Arm();
                arm.setDefaultCommand(new ArmCommand(arm, manualControls));

                autoBuilder = new AutoBuilder(drivetrain);

                driveCommand = new DriveCommand(drivetrain, autoBuilder, manualControls);
                drivetrain.setDefaultCommand(driveCommand);


                autoChooser = new SendableChooser<String>();

                // Add commands to the autonomous command chooser
                String defaultAuto = "Straight";
                autoChooser.setDefaultOption(defaultAuto, defaultAuto);

                // for (String f : (new File(Filesystem.getDeployDirectory().getPath() + "/pathplanner")).list()) {
                //         f = f.strip().replace(".path", "");
                //         if (!f.equals(defaultAuto)) {
                //                 autoChooser.addOption(f, f);
                //                 System.out.println(f);
                //         }
                // }

                autoChooser.addOption("StraightSpin", "StraightSpin");
                autoChooser.addOption("StraightAndBackSpin", "StraightAndBackSpin");
                autoChooser.addOption("StraightAndBack", "StraightAndBack");
                autoChooser.addOption("Hamid", "Hamid");
                autoChooser.addOption("HamidSharp", "HamidSharp");
                autoChooser.addOption("ChargingStationAuto", "ChargingStationAuto");

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
