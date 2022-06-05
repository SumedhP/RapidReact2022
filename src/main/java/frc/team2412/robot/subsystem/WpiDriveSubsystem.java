package frc.team2412.robot.subsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import static frc.team2412.robot.Hardware.*;

import org.frcteam2910.common.robot.drivers.PigeonTwo;

import com.swervedrivespecialties.swervelib.SwerveModule;

public class WpiDriveSubsystem extends SubsystemBase {
    
    private static final double TRACKWIDTH = Units.inchesToMeters(22.5);
    private static final double WHEELBASE = TRACKWIDTH;

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // front right
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // back left
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
    );

    private final SwerveModule[] modules;
    private final PigeonTwo gyro;

    public WpiDriveSubsystem(){
        modules = new SwerveModule[] { FRONT_LEFT_CONFIG.create(),
            FRONT_RIGHT_CONFIG.create(),
            BACK_LEFT_CONFIG.create(),
            BACK_RIGHT_CONFIG.create() };
            gyro = new PigeonTwo(GYRO_PORT, DRIVETRAIN_INTAKE_CAN_BUS_NAME);
    }

    @Override
    public void periodic() {
        
    }

    public void updateModules(){
        
    }

    SwerveControllerCommand cmd = new SwerveControllerCommand(null, null, null, null, null, null, null, null);

}
