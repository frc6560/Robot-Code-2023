// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team6560.frc2023.Constants;
import com.team6560.frc2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoBuilder {

  private HashMap<String, Command> eventMap;
  private List<PathPlannerTrajectory> pathGroup;

  private SwerveAutoBuilder autoBuilder;
  private Drivetrain drivetrain;

  public AutoBuilder(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("marker2", new PrintCommand("Passed marker 2"));

    autoBuilder = new SwerveAutoBuilder(
        () -> drivetrain.getPose(), // Pose2d supplier
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.5, 0.0), // PID constants to correct for translation error (used to create the X and Y
                                         // PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
                                         // controller)
        (state) -> drivetrain.setChassisState(state), // Module states consumer used to output to the drive subsystem
        eventMap,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following
                   // commands
    );

  }

  public Command getAutoCommand(String pathName) {
    // This will load the file "FullAuto.path" and generate it with a max velocity
    // of 2.0 m/s and a max acceleration of 1.0 m/s^2 for every path in the group
    pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(1.5, 0.75));

    drivetrain.resetOdometry(pathGroup.get(0).getInitialHolonomicPose());

    return new SequentialCommandGroup(
      autoBuilder.fullAuto(pathGroup)
    );

  }

}
