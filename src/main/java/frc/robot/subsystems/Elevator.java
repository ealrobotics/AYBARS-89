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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  // Drivetrain motors
  private final CANSparkMax m_leadMotor = new CANSparkMax(CANIDConstants.elevatorLeadMotorID,
      MotorType.kBrushless);

  private final CANSparkMax m_followMotor = new CANSparkMax(CANIDConstants.elevatorFollowMotorID,
      MotorType.kBrushless);

  private final SparkMaxPIDController m_PIDController;
  private final RelativeEncoder m_encoder;

  /** Creates a new Drive subsystem. */
  public Elevator() {
    // Reset motors
    m_leadMotor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();

    m_leadMotor.setIdleMode(IdleMode.kBrake);
    m_followMotor.setIdleMode(IdleMode.kBrake);

    m_PIDController = m_leadMotor.getPIDController();
    m_encoder = m_leadMotor.getEncoder();

    m_PIDController.setP(ElevatorConstants.kP);
    m_PIDController.setI(ElevatorConstants.kI);
    m_PIDController.setD(ElevatorConstants.kD);
    m_PIDController.setIZone(0);
    m_PIDController.setFF(0);
    m_PIDController.setOutputRange(ElevatorConstants.kMin, ElevatorConstants.kMax);
    m_leadMotor.setClosedLoopRampRate(0.5);

    m_followMotor.follow(m_leadMotor, true);

    m_leadMotor.burnFlash();
    m_followMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", m_encoder.getPosition());
  }

  public CommandBase runElevatorOpenLoop(double speed) {
    return run(() -> m_leadMotor.set(speed)).finallyDo((end) -> m_leadMotor.set(0.0));
  }

  public CommandBase runElevatorClosedLoop(double setpoint) {
    return run(() -> m_PIDController.setReference(setpoint, ControlType.kPosition))
        .until(() -> m_encoder.getPosition() == setpoint)
        .finallyDo((end) -> m_leadMotor.set(0.0));
  }
}