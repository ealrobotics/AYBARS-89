package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  DoubleSolenoid m_selenoid = new DoubleSolenoid(CANIDConstants.pcmID, PneumaticsModuleType.CTREPCM,
      GripperConstants.kGripperForward, GripperConstants.kGripperReverse);

  public Gripper() {
    m_selenoid.set(GripperConstants.kGripperDefaultState);
  }

  /**
   * Returns a command that toggles the Gripper solenoid
   */
  public CommandBase toggleGripper() {
    return runOnce(() -> m_selenoid.toggle())
        .withName("toggleGripper");
  }
}
