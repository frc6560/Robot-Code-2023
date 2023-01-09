package com.team6560.frc2023.utility;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The `Util` class provides a collection of utility methods that can be used in various parts of the codebase.
 */
public class Util {

    /**
     * Returns the given number, limited to the given maximum magnitude. If the given number is greater than the given
     * maximum magnitude, the maximum magnitude is returned. If the given number is less than the negative of the given
     * maximum magnitude, the negative of the maximum magnitude is returned. Otherwise, the given number is returned
     * unchanged.
     *
     * @param num the number to limit
     * @param maxMagnitude the maximum magnitude to allow
     * @return the limited value
     */
    public static double getLimited(double num, double maxMagnitude) {
        return getLimited(num, -maxMagnitude, maxMagnitude);
    }

    /**
     * Returns the given number, limited to the given minimum and maximum values. If the given number is greater than the
     * given maximum value, the maximum value is returned. If the given number is less than the given minimum value, the
     * minimum value is returned. Otherwise, the given number is returned unchanged.
     *
     * @param num the number to limit
     * @param min the minimum value to allow
     * @param max the maximum value to allow
     * @return the limited value
     */
    public static double getLimited(double num, double min, double max) {
        if (num > max) {
            return max;
        } else if (num < min) {
            return min;
        } else {
            return num;
        }
    }


    /**
     * Returns the given rotation, limited to the given maximum magnitude. If the given rotation is greater than the given
     * maximum magnitude, the maximum magnitude is returned. If the given rotation is less than the negative of the given
     * maximum magnitude, the negative of the maximum magnitude is returned. Otherwise, the given rotation is returned
     * unchanged.
     *
     * @param num the rotation to limit
     * @param maxMagnitude the maximum magnitude to allow
     * @return the limited value
     */
    public static Rotation2d getLimited(Rotation2d num, Rotation2d maxMagnitude) {
        return getLimited(num, maxMagnitude.unaryMinus(), maxMagnitude);
    }

    /**
     * Returns the given rotation, limited to the given minimum and maximum rotation. If the given rotation is greater than the
     * given maximum rotation, the maximum rotation is returned. If the given rotation is less than the given minimum rotation, the
     * minimum rotation is returned. Otherwise, the given rotation is returned unchanged.
     *
     * @param num the rotation to limit
     * @param min the minimum rotation to allow
     * @param max the maximum rotation to allow
     * @return the limited rotation
     */
    public static Rotation2d getLimited(Rotation2d num, Rotation2d min, Rotation2d max) {
        if (num.getDegrees() > max.getDegrees()) {
            return max;
        } else if (num.getDegrees() < min.getDegrees()) {
            return min;
        } else {
            return num;
        }
    }
    
    /**
     * Returns the difference in heading between the given headings. The returned value will be in the range (-180, 180].
     *
     * @param h1 the first heading
     * @param h2 the second heading
     * @return the difference in heading between the given headings
     */
    public static double getHeadingDiff(double h1, double h2) {
        double left = h1 - h2;
        double right = h2 - h1;
        if (left < 0) left += 360.0;
        if (right < 0) right += 360.0;
        return left < right ? -left : right;
    }

    /**
     * Returns the difference in heading between the given headings. The returned value will be in the range (-180, 180].
     *
     * @param h1 the first heading
     * @param h2 the second heading
     * @return the difference in heading between the given headings
     */
    public static Rotation2d getHeadingDiff(Rotation2d h1, Rotation2d h2) {
        Rotation2d left = h1.minus(h2);
        Rotation2d right = h2.minus(h1);
        if (left.getDegrees() < 0) left.plus(new Rotation2d(2*Math.PI));
        if (right.getDegrees() < 0) right.plus(new Rotation2d(2*Math.PI));
        return left.getDegrees() < right.getDegrees() ? left.unaryMinus() : right;
    }

}