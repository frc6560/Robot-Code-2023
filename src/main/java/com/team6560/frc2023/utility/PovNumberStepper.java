package com.team6560.frc2023.utility;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The `PovNumberStepper` class provides a utility for stepping up or down a
 * `NumberStepper` instance using the POV
 * (point-of-view) button on a joystick or Xbox controller.
 */
public class PovNumberStepper {
    public enum PovDirection {
        HORIZONTAL,
        VERTICAL
    }

    private final NumberStepper stepper;
    private Joystick joystick;
    private XboxController xbox;
    private final PovDirection direction;

    private int lastPov = -1;

    /**
     * Constructs a new `PovNumberStepper` instance that uses the given
     * `NumberStepper` instance and joystick for
     * controlling the number stepper with the joystick's POV button. The direction
     * of the POV button that will be used
     * for stepping up or down can be specified with the `direction` parameter.
     *
     * @param stepper   the `NumberStepper` instance to control
     * @param joystick  the joystick to use for input
     * @param direction the direction of the POV button to use for stepping up or
     *                  down
     */
    public PovNumberStepper(NumberStepper stepper, Joystick joystick, PovDirection direction) {
        this.stepper = stepper;
        this.joystick = joystick;
        this.direction = direction;

        CommandScheduler.getInstance().registerSubsystem(
                new Subsystem() {
                    @Override
                    public void periodic() {
                        update();
                    }
                });
    }

    /**
     * Constructs a new `PovNumberStepper` instance that uses the given
     * `NumberStepper` instance and Xbox controller for
     * controlling the number stepper with the Xbox controller's POV button. The
     * direction of the POV button that will be
     * used for stepping up or down can be specified with the `direction` parameter.
     *
     * @param stepper   the `NumberStepper` instance to control
     * @param xbox      the Xbox controller to use for input
     * @param direction the direction of the POV button to use for stepping up or
     *                  down
     */
    public PovNumberStepper(NumberStepper stepper, XboxController xbox, PovDirection direction) {
        this.stepper = stepper;
        this.xbox = xbox;
        this.direction = direction;

        CommandScheduler.getInstance().registerSubsystem(
                new Subsystem() {
                    @Override
                    public void periodic() {
                        update();
                    }
                });
    }

    /**
     * Updates the `NumberStepper` instance based on the current value of the
     * joystick or Xbox controller's POV button.
     * If the POV button is pressed in the specified direction, the number stepper
     * will be stepped up or down accordingly.
     */
    private void update() {
        int pov = joystick != null ? joystick.getPOV() : xbox.getPOV();

        if (lastPov == -1) {
            if (direction == PovDirection.VERTICAL) {
                if (pov == 0) {
                    stepper.stepUp();
                } else if (pov == 180) {
                    stepper.stepDown();
                }
            } else if (direction == PovDirection.HORIZONTAL) {
                if (pov == 90) {
                    stepper.stepUp();
                } else if (pov == 270) {
                    stepper.stepDown();
                }
            } else {
                throw new RuntimeException("unknown PovDirection value");
            }
        }
        lastPov = pov;
    }

    /**
     * Returns the current value of the underlying `NumberStepper` instance.
     *
     * @return the current value of the number stepper
     */
    public double get() {
        return stepper.get();
    }

    /**
     * Sets the current value of the underlying `NumberStepper` instance to the
     * given value. If the given value is outside
     * the defined range, it will be clamped to the nearest valid value.
     *
     * @param num the new value to set for the number stepper
     */
    public void set(double num) {
        stepper.set(num);
    }
}