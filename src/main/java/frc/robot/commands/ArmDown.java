package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmDown extends Command {
    private Arm m_arm;

    public ArmDown(Arm arm) {
        super();
        m_arm = arm;    
    }

    @Override
    public void execute() {
        m_arm.setPositionDown();
    }

    @Override
    public boolean isFinished() {
        return m_arm.getIsAtSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.stop();
    }
}
