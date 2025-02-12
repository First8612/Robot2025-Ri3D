package frc.robot.subsystems; 

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private SparkMax m_intakeMotor = new SparkMax(9, MotorType.kBrushless);
  private SparkMax m_intakeFollower = new SparkMax(10, MotorType.kBrushless);
  private DigitalInput m_sensor = new DigitalInput(0);
  private Timer m_sensorTimer = new Timer();
  private double m_speed;

  public Intake() {
    super();
    m_intakeMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeFollower.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeFollower.configure(new SparkMaxConfig().follow(m_intakeMotor, true), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_sensorTimer.reset();
  }

  public void intake(double speed) {
    m_speed = speed;
  }

  public void intakeWithSensor(double speed) {
    m_speed = speed;
    if (!m_sensor.get()) {
      m_sensorTimer.start();
      if (m_sensorTimer.hasElapsed(0.5)) {
        stop();
        m_sensorTimer.reset();
      }
    }
  }

  public boolean hasNote() {
    return !m_sensor.get();
  }

  public void stop() {
    m_speed = 0;
    m_intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
      m_intakeMotor.set(m_speed);
      SmartDashboard.putNumber("Intake Speed", m_intakeMotor.get());
  }
}
