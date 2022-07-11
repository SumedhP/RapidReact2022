package frc.team2412.robot.subsystem;

import static edu.wpi.first.math.kinematics.ChassisSpeeds.*;
import static frc.team2412.robot.Hardware.*;
import static frc.team2412.robot.subsystem.WpiDriveSubsystem.Constants.*;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;

public class WpiDriveSubsystem extends SubsystemBase {

    private final SwerveModule[] modules;
    private final WPI_Pigeon2 gyro;
    private final SwerveDriveOdometry odometry;

    public WpiDriveSubsystem() {
        modules = new SwerveModule[] { FRONT_LEFT_CONFIG.create(),
                FRONT_RIGHT_CONFIG.create(),
                BACK_LEFT_CONFIG.create(),
                BACK_RIGHT_CONFIG.create() };
        gyro = new WPI_Pigeon2(GYRO_PORT, DRIVETRAIN_INTAKE_CAN_BUS_NAME);
        odometry = new SwerveDriveOdometry(kinematics, getHeading());
    }

    @Override
    public void periodic() {
        odometry.update(getHeading(), getModuleStates());
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
        ChassisSpeeds chassisSpeeds = fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getHeading());
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        updateModules(moduleStates);
    }

    public void updateModules(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY_METERS_PER_SECOND);
        for (int i = 0; i < moduleStates.length; i++) {
            modules[i].set(moduleStates[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * 12,
                    moduleStates[i].angle.getRadians());
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleStates[i] = new SwerveModuleState(modules[i].getDriveVelocity(),
                    new Rotation2d(modules[i].getSteerAngle()));
        }
        return moduleStates;
    }

    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(pose, getHeading());
    }

    public void resetPose() {
        setPose(new Pose2d());
    }

    public static class Constants {
        public static final double TRACKWIDTH = Units.inchesToMeters(22.5);
        public static final double WHEELBASE = TRACKWIDTH;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
                new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // front right
                new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // back left
                new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
        );
    }

    public static class AutoUtil {

        public static final double maxAccelerationMetersPerSecondSquared = 3;
        public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

        private static final PIDController xController = new PIDController(1, 0, 0);
        private static final PIDController yController = new PIDController(1, 0, 0);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);

        private static final ProfiledPIDController thetaController = new ProfiledPIDController(
                1, 0, 0, kThetaControllerConstraints);

        private WpiDriveSubsystem driveSubsystem;

        public AutoUtil(WpiDriveSubsystem driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
        }

        public SwerveControllerCommand generateTrajectoryCommand(Trajectory trajectory) {
            driveSubsystem.setPose(trajectory.getInitialPose());
            return new SwerveControllerCommand(trajectory, driveSubsystem::getPose, kinematics, xController,
                    yController, thetaController, driveSubsystem::updateModules, driveSubsystem);
        }

    }
}
