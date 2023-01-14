// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team6560.frc2023.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.team6560.frc2023.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public static interface Controls {
    int getLimelightPipeline();
  }
    
  private final NetworkTableEntry ntX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
  private final NetworkTableEntry ntY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
  private final NetworkTableEntry ntV = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
  private final NetworkTableEntry ntL = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl");
  private final NetworkTableEntry ntBotPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
  private final NetworkTableEntry ntPipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline");

  private final Field2d field = new Field2d();

  private final Controls controls;
  private boolean forceOff = true;

  public Limelight(Controls controls) {
    this.controls = controls;
    setForceOff(false);

    ntDispTab("Limelight")
    .add("Horizontal Angle", this::getHorizontalAngle)
    .add("Vertical Angle", this::getVertAngle)
    .add("Has Target", this::hasTarget);
    ;

    SmartDashboard.putData("Field2", field);

  }

  public double getDistance() {
    // return -0.376023*ntY.getDouble(0.0) + 11.0681;
    return ntY.getDouble(0.0);
  }

  public double getHorizontalAngle() {
    return ntX.getDouble(0.0);
  }

  public double getVertAngle() {
    return ntY.getDouble(0.0);
  }

  public boolean hasTarget(){
    return ntV.getDouble(0.0) == 1.0;
  }

  public void setForceOff(boolean value) {
    forceOff = value;
  }

  public double getLatency() {
    // 11 additional ms is recommended for image capture latency
    // divided by 1000.0 to convert ms to s
    return (ntL.getDouble(0.0) + 11.0)/1000.0;
  }


  public Pair<Pose2d, Double> getBotPose() {

    if (!hasTarget()) return null;

    double currentTime = Timer.getFPGATimestamp() - getLatency();
    
    double[] da = ntBotPose.getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    if (da == null || da.length < 6) return null;

    Pose2d pose = new Pose3d(new Translation3d(da[0], da[1], da[2]), new Rotation3d(Math.toRadians(da[3]), Math.toRadians(da[4]), Math.toRadians(da[5]))).toPose2d();
    
    //transform pose from LL "field space" to pose2d
    pose = new Pose2d(pose.getTranslation().plus(new Translation2d(Constants.FieldConstants.length/2.0, Constants.FieldConstants.width/2.0)), pose.getRotation());

    // System.out.println("LL Field2d");
    // System.out.println(pose);

    field.setRobotPose(pose);
    
    return new Pair<Pose2d, Double> (pose, currentTime);
  }

  @Override
  public void periodic() {
    ntPipeline.setNumber(forceOff ? 0 : controls.getLimelightPipeline());
  }
}