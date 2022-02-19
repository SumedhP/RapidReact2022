package frc.team2412.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2412.robot.subsystem.ShooterSubsystem;

public class ShooterAimTestCommand extends InstantCommand {
    public ShooterAimTestCommand(ShooterSubsystem shooter) {
        super(() -> {
            System.out.println("bada bing bada boom");
            shooter.setFlywheelVelocity(shooter.getFlywheelTestVelocity());
            shooter.setHoodAngle(shooter.getHoodTestAngle());
        }, shooter);
    }
}
