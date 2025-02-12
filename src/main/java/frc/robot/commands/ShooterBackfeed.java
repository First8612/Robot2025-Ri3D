package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Shooter;

public class ShooterBackfeed extends SequentialCommandGroup {
    public ShooterBackfeed(Shooter shooter) {
        addRequirements(shooter);
        addCommands(
            new ConditionalCommand(
                new InstantCommand(), 
                new StartEndCommand(
                    shooter::backfeed,
                    shooter::stopFeed,
                    shooter
                )
                    .withTimeout(0.15)
                    .andThen(new InstantCommand(shooter::setHasBackfed)),
                shooter::getHasBackfed
            )
        );
    }
}
