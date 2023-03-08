package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

import java.util.HashMap;

public final class Autos {
  private final Drive m_drive;
  private final SendableChooser<Command> m_autonChooser;
  private final HashMap<String, Command> m_eventMap;
  private final RamseteAutoBuilder m_autonBuilder;

  public Autos(Drive drive) {
    m_drive = drive;

    m_eventMap = new HashMap<>();
    setMarkers();

    m_autonBuilder = new RamseteAutoBuilder(
        m_drive::getPose,
        m_drive::resetOdometry,
        m_drive.m_ramseteController,
        m_drive.m_kinematics,
        m_drive.m_feedforward,
        m_drive::getWheelSpeeds,
        new PIDConstants(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        m_drive::tankDriveVolts,
        m_eventMap,
        m_drive);

    m_autonChooser = new SendableChooser<Command>();
    m_autonChooser.setDefaultOption("No-op", new InstantCommand());
    m_autonChooser.addOption("Straight 3 Meters", this.FirstAuto());

    SmartDashboard.putData("Auton Chooser", m_autonChooser);
    PathPlannerServer.startServer(5811);
  }

  private void setMarkers() {
    // m_eventMap.put("Intake Stop", intake.stopIntake());
  }

  public Command getSelected() {
    return m_autonChooser.getSelected();
  }

  public Command FirstAuto() {
    return m_autonBuilder.fullAuto(PathPlanner.loadPath("straight-3m", AutoConstants.kConstraints));
  }
}