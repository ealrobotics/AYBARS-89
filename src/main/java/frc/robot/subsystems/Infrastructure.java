package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class Infrastructure extends SubsystemBase {
  private final Compressor m_compressor = new Compressor(CANIDConstants.pcmID, PneumaticsModuleType.CTREPCM);
  private final AddressableLED m_leds = new AddressableLED(0);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
  private int m_rainbowFirstPixelHue;

  private ledState m_ledState = ledState.RAINBOW;

  public Infrastructure() {
    m_compressor.enableDigital();
    m_leds.setLength(m_ledBuffer.getLength());
    m_leds.setData(m_ledBuffer);
    m_leds.start();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Compressor", m_compressor.isEnabled());
    SmartDashboard.putString("LED STATE", m_ledState.toString());
    updateLeds();
  }

  public enum ledState {
    CLOSED,
    CONE,
    CUBE,
    RAINBOW,
    RED,
    GREEN,
    BLUE
  }

  private void updateLeds() {
    if (m_ledState == ledState.RAINBOW) {
      ledsRainbow();
    }
    m_leds.setData(m_ledBuffer);
  }

  private void ledsRainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(
          i,
          (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180,
          255, 128);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
  }
}
