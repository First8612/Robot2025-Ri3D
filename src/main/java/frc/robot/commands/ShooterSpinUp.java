package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterSpinUp extends Command {
    private Shooter shooter;

    public ShooterSpinUp(Shooter shooter) {
        super();
        addRequirements(shooter);
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        super.initialize();

        shooter.setSpeakerShootSpeed();
    }

    @Override
    public boolean isFinished() {
        return shooter.isShooterAtSpeakerVelocity();
    }
}
