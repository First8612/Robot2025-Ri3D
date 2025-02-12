package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.NoteLimelight;

public class NoteAimController {
    private PIDController m_rotationAimController = new PIDController(0.009, 0.00001, 0.0040);
    private NoteLimelight limelight;

    public NoteAimController(NoteLimelight limelight) {
        super();
        this.limelight = limelight;
    }

    public void reset() {
        m_rotationAimController.reset();
    }

    public double calculate() {
        var rot = m_rotationAimController.calculate(limelight.getTargetX());
        rot = MathUtil.clamp(rot, -0.5, 0.5);

        return rot;
    }

    public PIDController getInternalController() {
        return m_rotationAimController;
    }
}
