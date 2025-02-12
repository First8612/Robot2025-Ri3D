package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ShooterPrep extends SequentialCommandGroup {
    public ShooterPrep(Shooter shooter) {
        super();
        addCommands(
            new ShooterBackfeed(shooter),
            new RunCommand(shooter::shootSpeakerOnly, shooter)
        );
    }
}
