// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.auton.Autos;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AybarsBot {
  // The robot's subsystems
  private final Drive m_drive = new Drive();
  // private final Elevator m_elevator = new Elevator();
  // private final Autos m_autos = new Autos(m_drive);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  /*
   * private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(4);
   * private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(4);
   */

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public void configureBindings() {
    /*
     * m_elevator.setDefaultCommand(
     * m_elevator.runElevatorOpenLoop(m_driverController.getLeftTriggerAxis()));
     * 
     * new
     * Trigger(m_driverController.a()).toggleOnTrue(m_elevator.runElevatorOpenLoop(0
     * .1));
     * new
     * Trigger(m_driverController.b()).toggleOnTrue(m_elevator.runElevatorOpenLoop(-
     * 0.1));
     */

    /*
     * m_drive.setDefaultCommand(
     * m_drive.arcadeDriveCommand(
     * () -> -m_speedLimiter.calculate(m_driverController.getLeftY()),
     * () -> -m_rotLimiter.calculate(m_driverController.getRightX())));
     */

    /*
     * m_drive.setDefaultCommand(
     * m_drive.driveWithSpeedsCommand(
     * -m_speedLimiter.calculate(m_driverController.getLeftY())
     * DriveConstants.kMaxSpeed,
     * -m_rotLimiter.calculate(m_driverController.getRightX()) *
     * DriveConstants.kMaxAngularSpeed));
     */

    // Boost robot speed when holding the right trigger (R2)
    new Trigger(m_driverController.rightTrigger(0))
        .onTrue(m_drive.boostCommand(false))
        .onFalse(m_drive.boostCommand(true));

    /*
     * Trigger Coast/Brake modes when DS is Disabled/Enabled
     * Set to Coast mode 5 seconds after disabling the robot.
     * Set to Brake mode when enabled.
     */
    new Trigger(DriverStation::isEnabled)
        .onFalse(
            new WaitCommand(5)
                .andThen(m_drive.setBrakeMode(false).ignoringDisable(true)))
        .onTrue(
            m_drive.setBrakeMode(true));
  }

  public Command getAutonomousCommand() {
    // return m_autos.getSelected();
    return new InstantCommand();
  }
}
