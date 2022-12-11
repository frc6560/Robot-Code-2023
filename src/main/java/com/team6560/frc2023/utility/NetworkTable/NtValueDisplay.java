// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2023.utility.NetworkTable;

import java.util.function.Supplier;

import com.team6560.frc2023.utility.AlwaysRunCommand;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The `NtValueDisplay` class provides a utility for displaying values on a
 * NetworkTables dashboard.
 */
public class NtValueDisplay<T> {
    final NetworkTable ntTable;
    final NetworkTableEntry entry;
    Supplier<T> value = null;

    /**
     * An interface that provides a helper method for creating a tab in the
     * NetworkTables dashboard and adding values to
     * display in that tab.
     */
    public interface DispHelper {
        /**
         * Adds a value to display in the specified NetworkTables tab.
         *
         * @param name  the name of the value to display
         * @param value the supplier that provides the value to display
         * @return this `DispHelper` instance
         */
        DispHelper add(String name, Supplier<Object> value);
    }

    /**
     * Displays the given value on the NetworkTables dashboard with the specified
     * name. The value will be displayed in
     * the "dashboard" tab.
     *
     * @param name  the name of the value to display
     * @param value the supplier that provides the value to display
     */
    public static void ntDisp(String name, Supplier<Object> value) {
        ntDisp("dashboard", name, value);
    }

    /**
     * Displays the given value on the NetworkTables dashboard with the specified
     * name and in the specified tab.
     *
     * @param tab   the name of the tab to display the value in
     * @param name  the name of the value to display
     * @param value the supplier that provides the value to display
     */
    public static void ntDisp(String tab, String name, Supplier<Object> value) {
        NtValueDisplay<Object> valueDisplay = new NtValueDisplay<>(tab, name);

        CommandScheduler.getInstance().schedule(new AlwaysRunCommand(() -> valueDisplay.update(value.get())));
    }

    /**
     * Creates a new `DispHelper` instance that will create a new NetworkTables tab
     * with the specified name.
     *
     * @param tab the name of the tab to create
     * @return a `DispHelper` instance that can be used to add values to the new tab
     */
    public static DispHelper ntDispTab(String tab) {
        return new DispHelper() {
            @Override
            public DispHelper add(String name, Supplier<Object> value) {
                ntDisp(tab, name, value);
                return this;
            }
        };
    }

    /**
     * Constructs a new `NtValueDisplay` instance that will display values in the
     * "dashboard" NetworkTables tab with the
     * specified name.
     *
     * @param name the name of the value to display
     */
    public NtValueDisplay(String name) {
        this("dashboard", name);
    }

    /**
     * Constructs a new `NtValueDisplay` instance that will display values in the
     * specified NetworkTables tab with the
     * specified name.
     *
     * @param tab  the name of the tab to display the value in
     * @param name the name of the value to display
     */
    public NtValueDisplay(String tab, String name) {
        ntTable = NetworkTableInstance.getDefault().getTable(tab);
        this.entry = ntTable.getEntry(name);
    }

    /**
     * Updates the displayed value with the given object.
     *
     * @param value the new value to display
     */
    public void update(Object value) {
        entry.setValue(value);
    }
}