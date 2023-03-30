// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.commands.auto;

import java.util.List;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.WaitBehavior;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team6560.frc2023.Constants;
import com.team6560.frc2023.subsystems.Arm;
import com.team6560.frc2023.subsystems.Drivetrain;
import com.team6560.frc2023.subsystems.Intake;
import com.team6560.frc2023.subsystems.Arm.ArmPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * 
 * The AutoBuilder class generates commands for autonomous actions using a
 * Drivetrain and path planning.
 * 
 * It uses a HashMap to store "marker" events and corresponding actions, as well
 * as a SwerveAutoBuilder for generating path following commands.
 */
public class AutoBuilder {

  /**
   * 
   * A HashMap containing "marker" events and corresponding actions.
   */
  private HashMap<String, Command> eventMap;


  /**
   * 
   * An instance of SwerveAutoBuilder used to generate path following commands.
   */
  private SwerveAutoBuilder autoBuilder;

  /**
   * 
   * An instance of Drivetrain that represents the drive subsystem.
   */
  private Drivetrain drivetrain;

  private Arm arm;
  private Intake intake;

  private SwerveAutoBuilder teleopAutoBuilder;

  /**
   * 
   * Constructor for AutoBuilder class. Initializes eventMap, autoBuilder, and
   * drivetrain.
   * 
   * @param drivetrain An instance of Drivetrain that represents the drive
   *                   ubsystem.
   */
  public AutoBuilder(Drivetrain drivetrain, Arm arm, Intake intake) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.intake = intake;

    eventMap = new HashMap<>();
    eventMap.put("printCommand", new PrintCommand("test Print command"));
    // eventMap.put("PLACE_CONE_HIGH", new RunCommand( () ->
    // arm.setArmState(ArmPose.HIGH_CONE), arm).until( () ->
    // arm.isArmAtSetpoint()));
    eventMap.put("PLACE_CONE_HIGH", new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, true));
    eventMap.put("PLACE_CONE_MID", new MoveArmToPoseCommand(arm, ArmPose.MEDIUM_CONE, true));
    eventMap.put("PLACE_CUBE_LOW", new MoveArmToPoseCommand(arm, ArmPose.LOW_CUBE, true));
    eventMap.put("PLACE_CONE_LOW", new MoveArmToPoseCommand(arm, ArmPose.LOW_CONE, true));
    eventMap.put("PLACE_CUBE_HIGH", new MoveArmToPoseCommand(arm, ArmPose.HIGH_CUBE, true));
    eventMap.put("PLACE_CUBE_MID", new MoveArmToPoseCommand(arm, ArmPose.MEDIUM_CUBE, true));
    eventMap.put("PICK_CUBE_GROUND", new MoveArmToPoseCommand(arm, ArmPose.GROUND_CUBE, true));

    eventMap.put("MOVE_CONE_HIGH", new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, false));
    eventMap.put("MOVE_CONE_MID", new MoveArmToPoseCommand(arm, ArmPose.MEDIUM_CONE, false));
    eventMap.put("MOVE_CUBE_LOW", new MoveArmToPoseCommand(arm, ArmPose.LOW_CUBE, false));
    eventMap.put("MOVE_CONE_LOW", new MoveArmToPoseCommand(arm, ArmPose.LOW_CONE, false));
    eventMap.put("MOVE_CUBE_HIGH", new MoveArmToPoseCommand(arm, ArmPose.HIGH_CUBE, false));
    eventMap.put("MOVE_CUBE_MID", new MoveArmToPoseCommand(arm, ArmPose.MEDIUM_CUBE, false));
    eventMap.put("MOVE_CUBE_GROUND", new MoveArmToPoseCommand(arm, ArmPose.GROUND_CUBE, false));

    eventMap.put("RETRACT_ARM", new MoveArmPistonCommand(arm, false));
    eventMap.put("INTAKE_CUBE", new IntakePickupAuto(intake, arm, true));
    eventMap.put("INTAKE_CONE", new IntakePickupAuto(intake, arm, false));


    eventMap.put("DEFAULT", new MoveArmToPoseCommand(arm, ArmPose.DEFAULT));
    
    drivetrain.setBatteryBullshit(false);

    autoBuilder = new SwerveAutoBuilder(
        () -> drivetrain.getOdometryPose2dNoApriltags(), // Pose2d supplier TODO: possibly revert back to no apriltags
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                          // subsystem
        eventMap,
        true,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );

    teleopAutoBuilder = new SwerveAutoBuilder(
        () -> drivetrain.getPose(), // Pose2d supplier
        (pose) -> drivetrain.resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.6, 0.1, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        (state) -> drivetrain.autoSetChassisState(state), // Module states consumer used to output to the drive
                                                          // subsystem
        eventMap,
        false,
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );

  }
  
  public Command getAutoCommand(String pathName) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(1.5, 0.75));

    return autoBuilder.fullAuto(pathGroup);
  }

  public  Command getInitiIntake(){
    return new IntakeInitAuto(intake, arm, false);
  }

  public Command getPlaceCone(){
    return new SequentialCommandGroup(
      new IntakeInitAuto(intake, arm, false),
      new MoveArmToPoseCommand(this.arm, ArmPose.HIGH_CONE, true),
      new MoveArmPistonCommand(this.arm, false),
      new MoveArmToPoseCommand(this.arm, ArmPose.DEFAULT, true)
    );
  }
  public Command getPlaceCube(){
    return new SequentialCommandGroup(
      new IntakeInitAuto(intake, arm, false),
      new MoveArmToPoseCommand(this.arm, ArmPose.HIGH_CONE, true),
      new MoveArmPistonCommand(this.arm, false),
      new MoveArmToPoseCommand(this.arm, ArmPose.DEFAULT, true)
    );
  }


  public Command getTwoBallLeft(){
    return getTwoBallLeft(2,2);
  }
  public Command getTwoBallLeft(double maxVelocity, double maxAcceleration){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TwoBallLeft", new PathConstraints(maxVelocity, maxAcceleration));
    
    return new SequentialCommandGroup(
      new IntakeInitAuto(intake, arm, false),

      new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, true),
      new MoveArmPistonCommand(this.arm, false),

      autoBuilder.fullAuto(pathGroup.get(0)).alongWith(
        new IntakePickupAuto(intake, arm, true)
      ),

      new MoveArmToPoseCommand(arm, ArmPose.HIGH_CUBE),
      new PrintCommand("Finished 2 ball biches")
    );
  }

  public Command getTwoBallLeftNew() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TwoBallLeftNew", new PathConstraints(2.5, 2.25));
    return new SequentialCommandGroup(
      new IntakeInitAuto(intake, arm, false),

      new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, true),
      // new MoveArmPistonCommand(this.arm, false),

      new InstantCommand(() -> arm.setArmExtention(false)),

      autoBuilder.fullAuto(pathGroup)
    );
  }

  public Command getTaxiCharge(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TaxiCharge", new PathConstraints(2, 1));

    return new SequentialCommandGroup(
      autoBuilder.fullAuto(pathGroup.get(0)),
      new ChargingStationAuto(drivetrain)
    );
  }

  public Command getPlaceTaxiChargeCone(){
    return getPlaceCone().andThen(getTaxiCharge());
  }
  public Command getPlaceTaxiChargeCube(){
    return getPlaceCube().andThen(getTaxiCharge());
  }

  public Command getTaxi(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Taxi", new PathConstraints(2, 1));

    return autoBuilder.fullAuto(pathGroup.get(0));
  }
  public Command getPlaceTaxiCone(){
    return getPlaceCone().andThen(getTaxi());
  }
  public Command getPlaceTaxiCube(){
    return getPlaceCube().andThen(getTaxi());
  }

  public Command getCharge(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Charge", new PathConstraints(1, 1));

    return new SequentialCommandGroup(
      autoBuilder.fullAuto(pathGroup.get(0)),
      new ChargingStationAuto(drivetrain)
    );
  }

  public Command getPlaceChargeCone(){  
    return getPlaceCone().andThen(getCharge());
  }
  public Command getPlaceChargeCube(){
    return getPlaceCube().andThen(getCharge());
  }

  public Command getTowBallChargeLeft(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("ChargeFromPlaceLeft", new PathConstraints(2, 2.25));

    return getTwoBallLeft(2.75,2.25).andThen(autoBuilder.fullAuto(pathGroup)).andThen(new ChargingStationAuto(drivetrain));
  }

  public Command getLoco(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TwoBallLeftLoco", new PathConstraints(2.5, 2.25));

    return new SequentialCommandGroup(
      new IntakeInitAuto(intake, arm, false),

      new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, true),
      // new MoveArmPistonCommand(this.arm, false),

      new InstantCommand(() -> arm.setArmExtention(false)),

      autoBuilder.fullAuto(pathGroup.get(0)),

      new ChargingStationAuto(drivetrain)
    );
  }

  public Command getTwoAndAHalfBallLoco(){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TwoAndAHalfBallLeftLoco", new PathConstraints(2.5, 2.25));

    return new SequentialCommandGroup(
      new IntakeInitAuto(intake, arm, false),

      new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, true),
      // new MoveArmPistonCommand(this.arm, false),

      new InstantCommand(() -> arm.setArmExtention(false)),

      autoBuilder.fullAuto(pathGroup.get(0))
    );
  }

  public Command getCableCarrierLoco() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("CableCarrierLoco", new PathConstraints(1.7, 1.5));

    return new SequentialCommandGroup(
      new IntakeInitAuto(intake, arm, false),

      new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, true),
      // new MoveArmPistonCommand(this.arm, false),

      new InstantCommand(() -> arm.setArmExtention(false)),

      autoBuilder.fullAuto(pathGroup.get(0))
    );
  }

  public Command getThreeBallLoco() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("ThreeBallLeftLoco", new PathConstraints(4.0, 3.5));

    return new SequentialCommandGroup(
      new IntakeInitAuto(intake, arm, false),

      new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, true),
      // new MoveArmPistonCommand(this.arm, false),

      new InstantCommand(() -> arm.setArmExtention(false)),

      autoBuilder.fullAuto(pathGroup.get(0)),

      new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, true),

      new InstantCommand(() -> arm.setArmExtention(false))
    );
  }

  public Command getChargingStationCableLoco() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("ChargingStationCableLoco", new PathConstraints(1.0, 1.0));

    return new SequentialCommandGroup(
      new IntakeInitAuto(intake, arm, false),

      new MoveArmToPoseCommand(arm, ArmPose.HIGH_CONE, true),
      // new InstantCommand(() -> arm.setArmExtention(false)),
      // new MoveArmToPoseCommand(this.arm, ArmPose.DEFAULT),

      autoBuilder.fullAuto(pathGroup.get(0)),

      new ChargingStationAuto(drivetrain)
    );
  }

  public Command getTestAutoCommand() {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test3", new PathConstraints(3.5, 2.0));


    return new SequentialCommandGroup(
        new IntakeInitAuto(intake, arm, false),
        new MoveArmToPoseCommand(this.arm, ArmPose.HIGH_CONE, true),
        new MoveArmPistonCommand(this.arm, false),
        new MoveArmToPoseCommand(this.arm, ArmPose.DEFAULT, true),
        autoBuilder.fullAuto(pathGroup),

        new ChargingStationAuto(drivetrain));
  }

  
  public Command goToPose(Pose2d desiredPose, Rotation2d heading) {

    Pose2d currPose = drivetrain.getPose();

    ChassisSpeeds currChassisSpeeds = drivetrain.getChassisSpeeds();

    double currSpeed = Math.abs(Math.hypot(currChassisSpeeds.vxMetersPerSecond, currChassisSpeeds.vyMetersPerSecond));

    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(2.0, 1.0),
        // position, heading(direction of travel), holonomic rotation, velocity verride
        new PathPoint(currPose.getTranslation(), heading, currPose.getRotation(), currSpeed),
        new PathPoint(desiredPose.getTranslation(), heading, desiredPose.getRotation()));

    return teleopAutoBuilder.followPath(traj);
  }

}
