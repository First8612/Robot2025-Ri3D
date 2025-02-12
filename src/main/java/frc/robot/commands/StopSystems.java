package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;

public class StopSystems extends ParallelCommandGroup {
    public StopSystems(Shooter shooter, Intake intake, Arm arm) {
        addRequirements(shooter);
        addRequirements(intake);
        addRequirements(arm);

        addCommands(
            new InstantCommand(() -> {shooter.stop();}),
            new InstantCommand(() -> {intake.stop();}),
            new InstantCommand(() -> {arm.setPositionDown();})
        );
    }
}
