package com.team6560.frc2023.auto;

import com.team6560.frc2023.subsystems.Drivetrain;

public class StraightenWheelsCommand extends SetWheelPoseCommand{

    public StraightenWheelsCommand(Drivetrain drivetrain) {
        super(drivetrain, 0.0, 0.0, 0.0, 0.0);
    }
    
}
