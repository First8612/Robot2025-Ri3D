package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private SparkMax m_climbMotor = new SparkMax(12, MotorType.kBrushless);
  private SparkMax m_climbFollower = new SparkMax(13, MotorType.kBrushless);
  private double m_speed;

    public Climber() {
      super();
      m_climbMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      m_climbFollower.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      m_climbMotor.getEncoder().setPosition(0);
      m_climbFollower.getEncoder().setPosition(0);
    }

    public void climbUp(double speed) {
      m_speed = speed;
    }

    public void climbDown(double speed) {
      m_speed = speed;
    }

    public void adjustSide(String side) {
      if (side.equals("left")) {
        m_climbMotor.set(0.3);
        m_climbFollower.stopMotor();
      } else if (side.equals("right")) {
        m_climbMotor.stopMotor();
        m_climbFollower.set(0.3);
      }
    }

    public void stop() {
      m_speed = 0;
      m_climbMotor.stopMotor();
      m_climbFollower.stopMotor();
    }

    @Override
    public void periodic() {
      m_climbMotor.set(m_speed);
      m_climbFollower.set(m_speed);
      SmartDashboard.putNumber("Left Climber Position", m_climbMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("Right Climber Position", m_climbFollower.getEncoder().getPosition());
    }


}
