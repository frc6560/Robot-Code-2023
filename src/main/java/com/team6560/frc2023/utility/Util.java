package com.team6560.frc2023.utility;

import edu.wpi.first.math.geometry.Rotation2d;

public class Util {
    public static double getLimited(double num, double maxMagnitude) {
        if (num > maxMagnitude) {
            return maxMagnitude;
        } else if (num < -maxMagnitude) {
            return -maxMagnitude;
        } else {
            return num;
        }
    }
    
    public static double getHeadingDiff(double h1, double h2) {
        double left = h1 - h2;
        double right = h2 - h1;
        if (left < 0) left += 360.0;
        if (right < 0) right += 360.0;
        return left < right ? -left : right;
    }

    public static Rotation2d getHeadingDiff(Rotation2d h1, Rotation2d h2) {
        Rotation2d left = h1.minus(h2);
        Rotation2d right = h2.minus(h1);
        if (left.getDegrees() < 0) left.plus(new Rotation2d(2*Math.PI));
        if (right.getDegrees() < 0) right.plus(new Rotation2d(2*Math.PI));
        return left.getDegrees() < right.getDegrees() ? left.unaryMinus() : right;
    }

    public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
    }
    
    public static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
