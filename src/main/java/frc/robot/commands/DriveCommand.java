package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {
    private final Drivetrain m_drivetrainSubsystem;


    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.MAX_ACCELERATION);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.MAX_ACCELERATION);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.MAX_ANGULAR_ACCELERATION);

    public static interface Controls {
        double driveX();
        double driveY();
        double driveRotation();
        boolean driveResetYaw();
    }

    private Controls controls;

    // X shape for defense
    public static final SwerveModuleState[] defaultState = new SwerveModuleState[] {
        new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(45))),
        new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(-45))),
        new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(-45))),
        new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(45)))
    };

    public DriveCommand(Drivetrain drivetrainSubsystem, Controls controls) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.controls = controls;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if (Math.abs(controls.driveX()) > 0 || Math.abs(controls.driveY()) > 0 || Math.abs(controls.driveRotation()) > 0 ) {
            m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xLimiter.calculate(controls.driveX()),
                        yLimiter.calculate(controls.driveY()),
                        turnLimiter.calculate(controls.driveRotation()),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
            );
        }
        else {
            // X shape for defense
            m_drivetrainSubsystem.setChassisState(defaultState);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
