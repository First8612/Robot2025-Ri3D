package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeAndFeed extends ParallelCommandGroup {
    public IntakeAndFeed(Shooter shooter, Intake intake) {
        addRequirements(shooter);
        addRequirements(intake);
        addCommands(
            new StartEndCommand(() -> {
                shooter.feedOnly();
            },
            () -> {
                shooter.stop();
            }),
            new StartEndCommand(() -> {
                intake.intake(1);
            },
            () -> {
                intake.stop();
            })
        );
    }
}
