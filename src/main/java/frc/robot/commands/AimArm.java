package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.TagLimelight;

public class AimArm extends Command {
    private TagLimelight limelight;
    private Arm arm;
    private boolean continuous;

    public AimArm(Arm arm, TagLimelight limelight, boolean continuous) {
        super();
        this.arm = arm;
        this.limelight = limelight;
        this.continuous = continuous;
    }

    @Override
    public boolean isFinished() {
        if (continuous) return false;

        return arm.getIsAtSetpoint();
    }

    @Override
    public void execute() {
        switch (limelight.getLastTargetType()) {
            case AMP:
                arm.setAimpointAmp();
                break;
            
            case SPEAKER:
                arm.setAimpointSpeaker(limelight);
                break;

            default:
                //nothing
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (continuous) {
            arm.setPositionDown();
        }
    }
}
