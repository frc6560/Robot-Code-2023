// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import java.util.Arrays;
import java.util.function.Supplier;

import com.team6560.frc2023.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public static interface Controls {
    int getLimelightPipeline();
  }

  private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    
  private final NetworkTableEntry ntX = networkTable.getEntry("tx");
  private final NetworkTableEntry ntY = networkTable.getEntry("ty");
  private final NetworkTableEntry ntV = networkTable.getEntry("tv");
  private final NetworkTableEntry ntL = networkTable.getEntry("tl");
  private final NetworkTableEntry ntcL = networkTable.getEntry("cl");
  private final NetworkTableEntry ntBotPose = networkTable.getEntry("botpose_wpiblue");
  private final NetworkTableEntry ntPipeline = networkTable.getEntry("pipeline");

  private final Field2d aprilTagField = new Field2d();

  // private final Field2d reflectiveTapeField = new Field2d();

  private final Controls controls;
  private boolean forceOff = true;

  private Supplier<Pose2d> predictedPose;

  public Limelight(Controls controls, Supplier<Pose2d> predictedPose) {
    this.controls = controls;
    this.predictedPose = predictedPose;
    
    setForceOff(false);

    ntDispTab("Limelight")
    .add("Horizontal Angle", this::getHorizontalAngle)
    .add("Vertical Angle", this::getVerticalAngle)
    .add("Has Target", this::hasTarget);

    SmartDashboard.putData("aprilTagField", aprilTagField);
    // SmartDashboard.putData("reflectiveTapeField", reflectiveTapeField);

  }


  public int getPipeline() {
    return controls.getLimelightPipeline();
  }

  public double getHorizontalAngle() {
    return ntX.getDouble(0.0);
  }

  public double getVerticalAngle() {
    return ntY.getDouble(0.0);
  }

  public boolean hasTarget(){
    return ntV.getDouble(0.0) == 1.0;
  }

  public void setForceOff(boolean value) {
    forceOff = value;
  }

  public double getEstimatedRobotXDistanceFromTargetMeters() {
    return convertAngleToHorizontalDistMeters(getHorizontalAngle());
  }

  public double getEstimatedRobotZDistanceFromTargetMeters() {
    return convertAngleToZDistMeters(getVerticalAngle());
  }

  public static double convertAngleToHorizontalDistMeters(double angleXDegrees) {
    return (Constants.FieldConstants.TOP_CONE_MARKER_TO_EDGE_Z_METERS / Math.tan(Math.toRadians(angleXDegrees))) - Constants.VisionConstants.LIMELIGHT_TO_ROBOT_X;
  }

  public static double convertAngleToZDistMeters(double angleYDegrees) {

    double LIMELIGHT_TO_TOP_CONE_MARKER_DISTANCE_METERS = Constants.FieldConstants.TOP_CONE_MARKER_TO_FLOOR_DISTANCE_METERS - Constants.VisionConstants.LIMELIGHT_TO_FLOOR_DISTANCE_METERS;

    return LIMELIGHT_TO_TOP_CONE_MARKER_DISTANCE_METERS / Math.tan(Math.toRadians(angleYDegrees));
  }

  public double getLatency() {
    // 11 additional ms is recommended for image capture latency
    // divided by 1000.0 to convert ms to s
    return (ntL.getDouble(0.0) + ntcL.getDouble(11.0))/1000.0;
  }


  


  public Pair<Pose2d, Double> getBotPose() {

    if (ntPipeline.getInteger(0l) != 0) return null;
    if (!hasTarget()) return null;

    double currentTime = Timer.getFPGATimestamp() - getLatency();
    
    double[] limelightBotPoseArray = ntBotPose.getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    if (limelightBotPoseArray == null || limelightBotPoseArray.length < 6) return null;

    if (new double[] {0.0, 0.0, 0.0, 0.0, 0.0}.equals(Arrays.copyOf(limelightBotPoseArray, limelightBotPoseArray.length - 1)))
      return null;
    
    Pose2d pose = new Pose3d(new Translation3d(limelightBotPoseArray[0], limelightBotPoseArray[1], limelightBotPoseArray[2]), new Rotation3d(Math.toRadians(limelightBotPoseArray[3]), Math.toRadians(limelightBotPoseArray[4]), Math.toRadians(limelightBotPoseArray[5]))).toPose2d();
    
    if (pose == null) return null;

    if (pose.equals(new Pose2d()))
      return null;
    //transform pose from LL "field space" to pose2d
    // pose = new Pose2d(pose.getTranslation().plus(new Translation2d(Constants.FieldConstants.length/2.0, Constants.FieldConstants.width/2.0)), pose.getRotation());

    // System.out.println("LL Field2d");
    // System.out.println(pose);

    aprilTagField.setRobotPose(pose);
    
    return new Pair<Pose2d, Double> (pose, currentTime);
  }

  @Override
  public void periodic() {
    ntPipeline.setNumber(forceOff ? 5 : controls.getLimelightPipeline());
  }
}
