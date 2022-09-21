package frc.team2412.robot.Subsystems;

import static frc.team2412.robot.Hardware.*;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

	SwerveModule[] modules;

	public Drivetrain() {

		modules = new SwerveModule[] { FRONT_LEFT_CONFIG.create(),
				FRONT_RIGHT_CONFIG.create(),
				BACK_LEFT_CONFIG.create(),
				BACK_RIGHT_CONFIG.create()};

	}

}
