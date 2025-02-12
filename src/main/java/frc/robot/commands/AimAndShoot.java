package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class AimAndShoot extends SequentialCommandGroup {
    public AimAndShoot(Shooter shooter, Arm arm, TagLimelight limelight) {
        addRequirements(shooter, arm, limelight);

        addCommands(
            new ParallelDeadlineGroup(
                new AimArm(arm, limelight, false),
                new ShooterPrep(shooter)
            ),
            new ShootSpeaker(shooter),
            new ArmDown(arm)
        );
    }
    
}
