// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  // Drivetrain motors
  private final CANSparkMax m_leadMotor = new CANSparkMax(CANIDConstants.elevatorLeadMotorID,
      MotorType.kBrushless);

  private final CANSparkMax m_followMotor = new CANSparkMax(CANIDConstants.elevatorFollowMotorID,
      MotorType.kBrushless);

  /** Creates a new Drive subsystem. */
  public Elevator() {

    // Reset motors
    m_leadMotor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();

    m_leadMotor.setSmartCurrentLimit(30);
    m_followMotor.setSmartCurrentLimit(30);

    m_leadMotor.setIdleMode(IdleMode.kCoast);
    m_followMotor.setIdleMode(IdleMode.kCoast);

    m_leadMotor.setInverted(ElevatorConstants.kLeadMotorInverted);
    m_followMotor.setInverted(ElevatorConstants.kFollowMotorInverted);

    m_followMotor.follow(m_leadMotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Speed", m_leadMotor.getAppliedOutput());
  }

  /**
   * Update odometry
   */
  public CommandBase runElevatorOpenLoop(double speed) {
    return run(() -> m_leadMotor.set(speed)).finallyDo((end) -> m_leadMotor.set(0.0));
  }
}
