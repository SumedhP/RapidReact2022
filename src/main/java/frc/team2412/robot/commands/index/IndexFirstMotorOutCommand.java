package frc.team2412.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystem.IndexSubsystem;

public class IndexFirstMotorOutCommand extends CommandBase {
    private final IndexSubsystem subsystem;

    public IndexFirstMotorOutCommand(IndexSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.firstMotorOut();
    }

    @Override
    public void execute() {
        if (!subsystem.firstSensorHasBallIn()) {
            subsystem.firstMotorStop();
        }
    }

    @Override
    public boolean isFinished() {
        return !subsystem.isFirstMotorOn();
    }
}
