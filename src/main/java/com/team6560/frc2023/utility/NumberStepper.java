package com.team6560.frc2023.utility;

/**
 * The `NumberStepper` class provides utility methods for stepping up or down a
 * number within a defined range.
 */
public class NumberStepper {
    private final double min;
    private final double max;
    private final double step;

    private double current;

    /**
     * Constructs a new `NumberStepper` instance with the given initial value,
     * minimum and maximum values, and step size.
     *
     * @param initial the initial value of the number stepper
     * @param min     the minimum value that the number stepper can have
     * @param max     the maximum value that the number stepper can have
     * @param step    the size of each step that the number stepper can take
     */
    public NumberStepper(double initial, double min, double max, double step) {
        this.min = min;
        this.max = max;
        this.step = step;

        current = initial;
    }

    /**
     * Constructs a new `NumberStepper` instance with the given minimum and maximum
     * values, and step size. The initial
     * value will be the minimum value.
     *
     * @param min  the minimum value that the number stepper can have
     * @param max  the maximum value that the number stepper can have
     * @param step the size of each step that the number stepper can take
     */
    public NumberStepper(double min, double max, double step) {
        this(min, min, max, step);
    }

    /**
     * Returns the current value of the number stepper.
     *
     * @return the current value of the number stepper
     */
    public double get() {
        return current;
    }

    /**
     * Sets the current value of the number stepper to the given value. If the given
     * value is outside the defined range,
     * it will be clamped to the nearest valid value.
     *
     * @param num the new value to set for the number stepper
     */
    public void set(double num) {
        current = num;
        limit();
    }

    /**
     * Steps the current value of the number stepper up by the step size. If the new
     * value is outside the defined range,
     * it will be clamped to the nearest valid value.
     */
    public void stepUp() {
        current += step;
        limit();
    }

    /**
     * Steps the current value of the number stepper down by the step size. If the
     * new value is outside the defined range,
     * it will be clamped to the nearest valid value.
     */
    public void stepDown() {
        current -= step;
        limit();
    }

    private void limit() {
        current = getClamped(current, min, max);
    }

    /**
     * Clamps the given value to the range defined by the minimum and maximum
     * values. If the given value is less than the
     * minimum value, the minimum value will be returned. If the given value is
     * greater than the maximum value, the maximum
     * value will be returned. Otherwise, the given value will be returned.
     *
     * @param num the value to clamp
     * @param min the minimum valid value
     * @param max the maximum valid value
     * @return the clamped value
     */
    private static double getClamped(double num, double min, double max) {
        if (num > max) {
            return max;
        } else if (num < min) {
            return min;
        } else {
            return num;
        }
    }
}