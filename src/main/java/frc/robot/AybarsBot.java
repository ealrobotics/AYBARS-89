// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.auton.Autos;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Infrastructure;
import frc.robot.subsystems.Pivot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AybarsBot {
  // The robot's subsystems
  private final Drive m_drive = new Drive();
  private final Elevator m_elevator = new Elevator();
  private final Pivot m_pivot = new Pivot();
  private final Gripper m_gripper = new Gripper();
  private final Infrastructure m_infrastructure = new Infrastructure();
  private final Autos m_autos = new Autos(m_drive);

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(DriveConstants.kSpeedRateLimit);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotRateLimit);

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public void configureBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);
    // Elevator Controls
    // Score High
    new Trigger(m_driverController.y())
        .onTrue(
            m_pivot.runPivotClosedLoop(Setpoints.kPivotHigh)
                .alongWith(m_elevator.runElevatorClosedLoop(Setpoints.kElevatorHigh)));

    // Score Mid
    new Trigger(m_driverController.b())
        .onTrue(
            m_pivot.runPivotClosedLoop(Setpoints.kPivotMid)
                .alongWith(m_elevator.runElevatorClosedLoop(Setpoints.kElevatorMid)));

    // Score Low
    new Trigger(m_driverController.a())
        .onTrue(
            m_pivot.runPivotClosedLoop(Setpoints.kPivotLow)
                .alongWith(m_elevator.runElevatorClosedLoop(Setpoints.kElevatorLow)));

    new Trigger(m_driverController.rightBumper())
        .onTrue(m_elevator.runElevatorClosedLoop(Setpoints.kElevatorLow));

    // Gripper Controls
    new Trigger(m_driverController.rightBumper()).onTrue(m_gripper.toggleGripper());

    // Drivetrain Controls
    /*
     * m_drive.setDefaultCommand(
     * m_drive.arcadeDrive(
     * () -> -m_speedLimiter.calculate(m_driverController.getLeftY()),
     * () -> -m_rotLimiter.calculate(m_driverController.getRightX()),
     * () -> m_driverController.rightTrigger(0).getAsBoolean()));
     */

    m_drive.setDefaultCommand(
        m_drive.driveWithSpeeds(
            () -> -m_speedLimiter.calculate(m_driverController.getLeftY()) * DriveConstants.kMaxSpeed,
            () -> -m_rotLimiter.calculate(m_driverController.getRightX()) * DriveConstants.kMaxAngularSpeed,
            () -> m_driverController.rightTrigger(0).getAsBoolean()));

    // Trigger Coast/Brake modes when DS is Disabled/Enabled
    new Trigger(DriverStation::isEnabled)
        .onFalse(
            new WaitCommand(5)
                .andThen(m_drive.setBrakeMode(false).ignoringDisable(true)))
        .onTrue(
            m_drive.setBrakeMode(true));
  }

  public Command getAutonomousCommand() {
    return m_autos.getSelected();
  }
}