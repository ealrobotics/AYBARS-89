// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AybarsBot {
  // The robot's subsystems
  private final Drive m_drive = new Drive();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // An example trajectory to follow during the autonomous period.
  private PathPlannerTrajectory m_trajectory;

  // Create Field2d for robot and trajectory visualizations.
  private Field2d m_field;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public void configureBindings() {
    // Boost robot speed when holding the right trigger (R2)
    new Trigger(m_driverController.rightTrigger(0))
        .onTrue(m_drive.boostCommand(true))
        .onFalse(m_drive.boostCommand(false));

    // Control the drive with split-stick arcade controls
    /*
     * m_drive.setDefaultCommand(
     * m_drive.arcadeDriveCommand(
     * () -> m_driverController.getLeftY(), () -> m_driverController.getRightX()));
     */

    m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(
            () -> -m_speedLimiter.calculate(m_driverController.getLeftY()),
            () -> -m_rotLimiter.calculate(m_driverController.getRightX())));

    /*
     * m_drive.setDefaultCommand(
     * m_drive.driveCommand(-m_speedLimiter.calculate(m_driverController.getLeftY())
     * * DriveConstants.kMaxSpeed,
     * -m_rotLimiter.calculate(m_driverController.getRightX()) *
     * DriveConstants.kMaxAngularSpeed));
     */
  }

  public void loadAutoPaths() {
    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    m_trajectory = PathPlanner.loadPath("straight-1",
        new PathConstraints(AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration));

    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    m_field.getObject("traj").setTrajectory(m_trajectory);
  }

  public CommandBase getAutonomousCommand() {
    m_drive.resetOdometryCommand(m_trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return m_drive.ramseteCommand(m_trajectory).andThen(() -> m_drive.tankDriveVolts(0, 0), m_drive);
  }
}
