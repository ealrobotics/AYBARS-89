package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class Infrastructure extends SubsystemBase {
  private final Compressor m_compressor = new Compressor(CANIDConstants.pcmID, PneumaticsModuleType.CTREPCM);

  public Infrastructure() {
    m_compressor.enableDigital();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Compressor", m_compressor.isEnabled());
  }
}
