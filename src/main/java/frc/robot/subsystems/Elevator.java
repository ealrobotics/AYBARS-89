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

  private final CANSparkMax m_pivotMotor = new CANSparkMax(CANIDConstants.elevatorPivotMotorID,
      MotorType.kBrushless);

  private final SparkMaxPIDController m_pivotPidController;
  private final RelativeEncoder m_pivotEncoder;

  /** Creates a new Drive subsystem. */
  public Elevator() {

    // Reset motors
    m_leadMotor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();
    m_pivotMotor.restoreFactoryDefaults();

    m_leadMotor.setIdleMode(IdleMode.kBrake);
    m_followMotor.setIdleMode(IdleMode.kBrake);
    m_pivotMotor.setIdleMode(IdleMode.kBrake);

    m_leadMotor.setInverted(ElevatorConstants.kLeadMotorInverted);
    m_followMotor.setInverted(ElevatorConstants.kFollowMotorInverted);
    m_pivotMotor.setInverted(ElevatorConstants.kPivotMotorInverted);

    m_pivotPidController = m_pivotMotor.getPIDController();
    m_pivotEncoder = m_pivotMotor.getEncoder();

    m_pivotPidController.setP(ElevatorConstants.kPivotP);
    m_pivotPidController.setI(0);
    m_pivotPidController.setD(ElevatorConstants.kPivotD);
    m_pivotPidController.setIZone(0);
    m_pivotPidController.setFF(0);

    m_pivotPidController.setOutputRange(ElevatorConstants.kPivotMin, ElevatorConstants.kPivotMax);

    SmartDashboard.putNumber("Set Rotations", 10);
    // m_followMotor.follow(m_leadMotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Speed", m_pivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Position", m_pivotEncoder.getPosition());
  }

  /**
   * Update odometry
   */
  public CommandBase runElevatorOpenLoop(double speed) {
    return run(() -> m_leadMotor.set(speed)).finallyDo((end) -> m_leadMotor.set(0.0));
  }

  public CommandBase runElevatorPivotOpenLoop(double speed) {
    return run(() -> m_pivotMotor.set(speed)).finallyDo((end) -> m_pivotMotor.set(0.0));
  }

  public CommandBase runElevatorPivotClosedLoop(double setpoint) {
    return run(() -> m_pivotPidController.setReference(setpoint, ControlType.kPosition))
        .until(() -> m_pivotEncoder.getPosition() == setpoint)
        .finallyDo((end) -> m_pivotMotor.set(0.0));
  }
}