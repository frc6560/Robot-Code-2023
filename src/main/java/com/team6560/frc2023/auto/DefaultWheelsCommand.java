package com.team6560.frc2023.auto;

import com.team6560.frc2023.subsystems.Drivetrain;

public class DefaultWheelsCommand extends SetWheelPoseCommand {

    public DefaultWheelsCommand(Drivetrain drivetrain) {
        super(drivetrain, 45.0, -45.0, -45.0, 45.0);
    }
    
}
