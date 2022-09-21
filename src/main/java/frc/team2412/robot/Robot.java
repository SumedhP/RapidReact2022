// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2412.robot;

import static frc.team2412.robot.Hardware.*;

import static java.lang.Thread.sleep;

import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

/* goals
 * Updates:
 * Timesplice robot
 * Odometry faster
 * Turret logic separate
 * Indexing a state machine
 * autos using path planner
 * 
 * things i want happening more often
 * DB update
 * Index
 * DB odometry
 */
public class Robot extends TimesliceRobot {

    private PowerDistribution PDP;
    private PneumaticHub pneumaticHub;

    private static final double MIN_PRESSURE = 90;
    private static final double MAX_PRESSURE = 110;

    public RobotContainer subsystems;

    public Robot() {
        super(0.002, 0.005);
        subsystems = new RobotContainer();
    }

    public double getVoltage() {
        return PDP.getVoltage();
    }

    // TODO add other override methods

    public Field2d field = new Field2d();

    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();

        PDP = new PowerDistribution(Hardware.PDP_ID, ModuleType.kRev);
        pneumaticHub = new PneumaticHub(PNEUMATIC_HUB);
        pneumaticHub.enableCompressorAnalog(MIN_PRESSURE, MAX_PRESSURE);

        Shuffleboard.startRecording();

        if (RobotBase.isReal()) {
            DataLogManager.start();
            DriverStation.startDataLog(DataLogManager.getLog(), true);
        }

        // Create and push Field2d to SmartDashboard.
        SmartDashboard.putData(field);

    }

    @Override
    public void testInit() {

    }

    @Override
    public void robotPeriodic() {
        Logger.updateEntries();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        Shuffleboard.startRecording();
    }

    @Override
    public void teleopInit() {
        Shuffleboard.startRecording();
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledInit() {
        Shuffleboard.stopRecording();
    }

    @Override
    public void disabledPeriodic() {

    }
}
