package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(CANIDConstants.elevatorPivotMotorID,
      MotorType.kBrushless);

  private final SparkMaxPIDController m_PIDController;
  private final RelativeEncoder m_encoder;

  public Pivot() {
    m_motor.restoreFactoryDefaults();
    m_motor.setSmartCurrentLimit(35);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(PivotConstants.kInverted);

    m_PIDController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    m_PIDController.setP(PivotConstants.kP);
    m_PIDController.setI(PivotConstants.kI);
    m_PIDController.setD(PivotConstants.kD);
    m_PIDController.setIZone(0);
    m_PIDController.setFF(0);
    m_PIDController.setOutputRange(PivotConstants.kMin, PivotConstants.kMax);
    m_motor.setClosedLoopRampRate(PivotConstants.kRampRate);

    m_motor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Pivot Speed", m_encoder.getVelocity());
  }

  public CommandBase runPivotOpenLoop(double speed) {
    return run(() -> m_motor.set(speed)).finallyDo((end) -> m_motor.set(0.0));
  }

  public CommandBase runPivotClosedLoop(double setpoint) {
    return run(() -> m_PIDController.setReference(setpoint, ControlType.kPosition))
        .until(() -> m_encoder.getPosition() == setpoint)
        .finallyDo((end) -> m_motor.set(0.0));
  }
}
