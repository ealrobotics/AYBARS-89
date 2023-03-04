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

  private final SparkMaxPIDController m_pivotPIDController;
  private final RelativeEncoder m_pivotEncoder;

  private final SparkMaxPIDController m_elevatorPIDController;
  private final RelativeEncoder m_elevatorEncoder;

  /** Creates a new Drive subsystem. */
  public Elevator() {
    // Reset motors
    m_leadMotor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();
    m_pivotMotor.restoreFactoryDefaults();

    m_leadMotor.setIdleMode(IdleMode.kBrake);
    m_followMotor.setIdleMode(IdleMode.kBrake);
    m_pivotMotor.setIdleMode(IdleMode.kBrake);

    m_pivotMotor.setInverted(ElevatorConstants.kPivotMotorInverted);

    m_pivotPIDController = m_pivotMotor.getPIDController();
    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_pivotEncoder.setPosition(0);
    m_pivotPIDController.setP(ElevatorConstants.kPivotP);
    m_pivotPIDController.setI(0);
    m_pivotPIDController.setD(ElevatorConstants.kPivotD);
    m_pivotPIDController.setIZone(0);
    m_pivotPIDController.setFF(0);
    m_pivotPIDController.setOutputRange(ElevatorConstants.kPivotMin, ElevatorConstants.kPivotMax);

    m_elevatorPIDController = m_leadMotor.getPIDController();
    m_elevatorEncoder = m_leadMotor.getEncoder();
    m_elevatorEncoder.setPosition(0);
    m_elevatorPIDController.setP(ElevatorConstants.kP);
    m_elevatorPIDController.setI(0);
    m_elevatorPIDController.setD(ElevatorConstants.kD);
    m_elevatorPIDController.setIZone(0);
    m_elevatorPIDController.setFF(0);
    m_elevatorPIDController.setOutputRange(ElevatorConstants.kMin, ElevatorConstants.kMax);

    SmartDashboard.putNumber("Set Rotations", 10);

    m_followMotor.follow(m_leadMotor, true);

    m_leadMotor.burnFlash();
    m_followMotor.burnFlash();
    m_pivotMotor.burnFlash();
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
    return run(() -> m_pivotPIDController.setReference(setpoint, ControlType.kPosition))
        .until(() -> m_pivotEncoder.getPosition() == setpoint)
        .finallyDo((end) -> m_pivotMotor.set(0.0));
  }

  public CommandBase runElevatorClosedLoop(double setpoint) {
    return run(() -> m_elevatorPIDController.setReference(setpoint, ControlType.kPosition))
        .until(() -> m_elevatorEncoder.getPosition() == setpoint)
        .finallyDo((end) -> m_leadMotor.set(0.0));
  }
}