package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private SparkMax m_armMotor = new SparkMax(16, MotorType.kBrushless);
    private RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
    private ArmAimHelper m_armAimHelper = new ArmAimHelper();
    private NetworkTableEntry m_armSetpointAdjustmentEntry;
    private PIDController m_positionController = new PIDController(0.15, 0.65, 0.001);
    private boolean m_stopped;

    // be careful setting this to low (slow) because it also prevents slowing down.
    private SlewRateLimiter m_rateLimiter = new SlewRateLimiter(6);

    private final double MAX_SETPOINT = 26;
    private final double AIM_ADJUSTMENT = 0.05;


    public Arm() {
        super();
        m_armMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_armMotor.getEncoder().setPosition(0);
        m_positionController.setIZone(2);
        setPositionDown();

        SmartDashboard.putData("Arm/Position Controller", m_positionController);
        SmartDashboard.putNumber("Arm/Position Adjustment", 0);

        m_armSetpointAdjustmentEntry = SmartDashboard.getEntry("Arm/Position Adjustment");
        m_armSetpointAdjustmentEntry.setDefaultDouble(0);
    }

    public void autoAimAdjustDown() {
        // name appears reversed, but to aim lower, we need to bump our setpoint up
        m_armSetpointAdjustmentEntry.setDouble(m_armSetpointAdjustmentEntry.getDouble(0) + AIM_ADJUSTMENT);
    }

    public void autoAimAdjustUp() {
        // name appears reversed, but to aim lower, we need to bump our setpoint up
        m_armSetpointAdjustmentEntry.setDouble(m_armSetpointAdjustmentEntry.getDouble(0) - AIM_ADJUSTMENT);
    }

    public void setPositionDown() {
        m_positionController.setSetpoint(0);
        m_stopped = false;
    }

    public void setAimpointAmp() {
        m_positionController.setSetpoint(MAX_SETPOINT);
        m_stopped = false;
    }

    public void adjustAim(double rate) {
        m_positionController.setSetpoint(
                MathUtil.clamp(m_positionController.getSetpoint() + rate, 0, MAX_SETPOINT));
    }

    public void setAimpointSpeaker(TagLimelight limelight) {
        var setpointAndSpeed = m_armAimHelper.getArmSetpoint(limelight);
        var setpoint = setpointAndSpeed.getFirst() * (1 + m_armSetpointAdjustmentEntry.getDouble(0));
        setpoint = MathUtil.clamp(setpoint, 0, MAX_SETPOINT);

        m_positionController.setSetpoint(setpoint);
        m_stopped = false;
    }

    public void aimDelivery() {
        m_positionController.setSetpoint(20);
        m_stopped = false;
    }

    public boolean getIsAtSetpoint() {
        return Math.abs(m_positionController.getPositionError()) < 0.15;
    }

    public void stop() {
        m_stopped = true;
        m_armMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if (!m_stopped) {
            var position = m_armEncoder.getPosition();
            var speed = m_positionController.calculate(position);
            speed = MathUtil.clamp(speed, -.25, 0.5);
            speed = m_rateLimiter.calculate(speed);

            if ((position < 8 && speed < 0) || (position > 42 && speed > 0)) {
                speed = MathUtil.clamp(speed, -.10, 0.25);
            }

            m_armMotor.setVoltage(speed * 12);
        }

        SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Setpoint", m_positionController.getSetpoint());
        SmartDashboard.putNumber("Arm Setpoint Error", m_positionController.getPositionError());
        SmartDashboard.putNumber("Arm Speed", m_armMotor.get());
        SmartDashboard.putBoolean("Arm Is At Setpoint", getIsAtSetpoint());
    }
}
