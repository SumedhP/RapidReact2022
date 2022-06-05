package frc.team2412.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team2412.robot.Hardware.*;

import com.swervedrivespecialties.swervelib.SwerveModule;

public class WpiDriveSubsystem extends SubsystemBase {
    
    private final SwerveModule[] modules;

    public WpiDriveSubsystem(){
        modules = new SwerveModule[] { FRONT_LEFT_CONFIG.create(),
            FRONT_RIGHT_CONFIG.create(),
            BACK_LEFT_CONFIG.create(),
            BACK_RIGHT_CONFIG.create() };
    }


}
