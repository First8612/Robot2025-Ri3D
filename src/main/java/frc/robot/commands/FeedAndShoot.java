package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;

public class FeedAndShoot extends ParallelCommandGroup {
    public FeedAndShoot(Shooter shooter, Intake intake) {
        addRequirements(shooter);
        addRequirements(intake);
        addCommands(
            new RunCommand(() -> {shooter.shootSpeakerOnly();}),
            new RunCommand(() -> {intake.intake(0.7);})
        );
    }
}
