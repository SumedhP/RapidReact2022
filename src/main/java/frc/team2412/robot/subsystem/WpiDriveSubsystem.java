package frc.team2412.robot.subsystem;

import static frc.team2412.robot.Hardware.*;

import org.frcteam2910.common.robot.drivers.PigeonTwo;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class WpiDriveSubsystem extends SubsystemBase {

    private static final double TRACKWIDTH = Units.inchesToMeters(22.5);
    private static final double WHEELBASE = TRACKWIDTH;
    private static final double maxVelocityMetersPerSecond = 1;

    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // front right
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // back left
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
    );

    private final SwerveModule[] modules;
    private final PigeonTwo gyro;

    private final SwerveDriveOdometry odometry;

    public WpiDriveSubsystem() {
        modules = new SwerveModule[] { FRONT_LEFT_CONFIG.create(),
                FRONT_RIGHT_CONFIG.create(),
                BACK_LEFT_CONFIG.create(),
                BACK_RIGHT_CONFIG.create() };
        gyro = new PigeonTwo(GYRO_PORT, DRIVETRAIN_INTAKE_CAN_BUS_NAME);
        odometry = new SwerveDriveOdometry(kinematics, getHeading());
    }

    @Override
    public void periodic() {
        odometry.update(getHeading(), getModuleStates());
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocityMetersPerSecond);
        updateModules(swerveModuleStates);
    }

    public void updateModules(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < 4; i++) {
            modules[i].set(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getRadians());
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = new SwerveModuleState(modules[i].getDriveVelocity(),
                    new Rotation2d(modules[i].getSteerAngle()));
        }
        return moduleStates;
    }

    public Rotation2d getHeading() {
        return new Rotation2d(gyro.getAdjustmentAngle().toRadians());
    }

    public void resetGyro() {
        gyro.setAdjustmentAngle(gyro.getUnadjustedAngle());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(pose, getHeading());
    }

    public void resetPose() {
        odometry.resetPosition(new Pose2d(), getHeading());
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
