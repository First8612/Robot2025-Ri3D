package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends SubsystemBase {
    private AddressableLED m_leds = new AddressableLED(0);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    String color = "";

    public LEDs() {
        m_leds.setLength(m_ledBuffer.getLength());
        m_leds.start();
    }

    public void setRed() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0);
         }
         m_leds.setData(m_ledBuffer);
         color = "Red";
    }

    public void setGreen() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 255, 0);
         }
         m_leds.setData(m_ledBuffer);
         color = "Green";
    }

    public void setBlue() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 255);
         }
         m_leds.setData(m_ledBuffer);
         color = "Blue";
    }

    @Override
    public void periodic() {
        m_leds.setData(m_ledBuffer);
        SmartDashboard.putString("LED Color", color);
    }

}
