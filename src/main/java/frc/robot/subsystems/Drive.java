// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends SubsystemBase {
  // Drivetrain motors
  private final CANSparkMax m_leftLeadMotor = new CANSparkMax(CANIDConstants.drivebaseLeftLeadMotorID,
      MotorType.kBrushed);
  private final WPI_VictorSPX m_leftFollowMotor = new WPI_VictorSPX(CANIDConstants.drivebaseLeftFollowMotorID);

  private final CANSparkMax m_rightLeadMotor = new CANSparkMax(CANIDConstants.drivebaseRightLeadMotorID,
      MotorType.kBrushed);
  private final WPI_VictorSPX m_rightFollowMotor = new WPI_VictorSPX(CANIDConstants.drivebaseRightFollowMotorID);

  private final MotorControllerGroup m_leftMotorControllerGroup = new MotorControllerGroup(m_leftFollowMotor,
      m_leftFollowMotor);

  private final MotorControllerGroup m_rightMotorControllerGroup = new MotorControllerGroup(m_rightFollowMotor,
      m_rightFollowMotor);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotorControllerGroup,
      m_rightMotorControllerGroup);

  // The left-side drive encoder
  private final Encoder m_leftEncoder = new Encoder(
      DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder = new Encoder(
      DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1],
      DriveConstants.kRightEncoderReversed);

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // PID Controllers
  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kLeftP, DriveConstants.kLeftI,
      DriveConstants.kLeftD);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kRightP, DriveConstants.kRightI,
      DriveConstants.kRightD);

  public final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      DriveConstants.kTrackWidthMeters);

  public final RamseteController m_ramseteController = new RamseteController();

  private final DifferentialDrivePoseEstimator m_poseEstimator;

  public final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV);

  private final Field2d m_field;

  private LimelightResults m_limelight;

  /** Creates a new Drive subsystem. */
  public Drive() {
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    // Reset encoders
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    // Reset motors
    m_leftLeadMotor.restoreFactoryDefaults();
    m_leftFollowMotor.configFactoryDefault();
    m_rightLeadMotor.restoreFactoryDefaults();
    m_rightFollowMotor.configFactoryDefault();

    m_leftMotorControllerGroup.setInverted(DriveConstants.kLeftLeadMotorInverted);
    m_rightMotorControllerGroup.setInverted(DriveConstants.kRightLeadMotorInverted);

    // TODO: set current limit for VictorSPX controllers too
    m_leftLeadMotor.setSmartCurrentLimit(30);
    // m_leftFollowMotor.setSmartCurrentLimit(30);
    m_rightLeadMotor.setSmartCurrentLimit(30);
    // m_rightFollowMotor.setSmartCurrentLimit(30);

    m_leftLeadMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollowMotor.setNeutralMode(NeutralMode.Brake);
    m_rightLeadMotor.setIdleMode(IdleMode.kBrake);
    m_rightFollowMotor.setNeutralMode(NeutralMode.Brake);

    m_drive.setMaxOutput(DriveConstants.kBoostedMaxSpeedPercentage);

    m_poseEstimator = new DifferentialDrivePoseEstimator(
        m_kinematics, Rotation2d.fromDegrees(m_gyro.getAngle()), m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(), new Pose2d());

    m_field = new Field2d();
    SmartDashboard.putData("robot_pose", m_field);

    // m_limelight = LimelightHelpers.getLatestResults("");
  }

  @Override
  public void periodic() {
    // Update odometry
    updateOdometry();
    updateField();
  }

  /**
   * Update odometry
   */
  public void updateOdometry() {
    m_poseEstimator.update(Rotation2d.fromDegrees(m_gyro.getAngle()), m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());

    /*
     * m_limelight = LimelightHelpers.getLatestResults("");
     * 
     * Pose2d limelight_botPose;
     * if (Constants.alliance == Alliance.Blue)
     * limelight_botPose = m_limelight.targetingResults.getBotPose2d_wpiBlue();
     * else
     * limelight_botPose = m_limelight.targetingResults.getBotPose2d_wpiRed();
     */

    /*
     * Filter vision pose
     * - Check tv (Valid Targets) != 0
     * - Check distance between known robot pose and vision pose < 1
     */
    /*
     * if (limelight_botPose.getTranslation().getDistance(m_poseEstimator.
     * getEstimatedPosition().getTranslation()) < 1.0
     * && m_limelight.targetingResults.valid) {
     * double limelight_latency = m_limelight.targetingResults.latency_pipeline
     * + m_limelight.targetingResults.latency_jsonParse;
     * 
     * m_poseEstimator.addVisionMeasurement(limelight_botPose,
     * Timer.getFPGATimestamp() - limelight_latency);
     * }
     */
  }

  /**
   * Update SmartDashboard field widget
   */
  public void updateField() {
    m_field.setRobotPose(getPose());
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Returns the pose of the robot.
   *
   * @return The pose of the robot.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the field-relative position to a specific location.
   *
   * @param pose The position to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * 
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotorControllerGroup.setVoltage(leftVolts);
    m_rightMotorControllerGroup.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * 
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Sets the desired wheel speeds.
   * 
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftMotorControllerGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightMotorControllerGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  public CommandBase driveWithSpeedsCommand(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    return run(() -> setSpeeds(wheelSpeeds));
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  /**
   * Returns a command that boosts robot speed
   * 
   * @param boost make it fast / or not
   */
  public CommandBase boostCommand(boolean boost) {
    return runOnce(
        () -> {
          m_drive.setMaxOutput(
              boost ? DriveConstants.kBoostedMaxSpeedPercentage : DriveConstants.kMaxSpeedPercentage);
        })
        .withName("Boost");
  }

  /**
   * Sets idle mode to be either brake mode or coast mode.
   * 
   * @param brake If true, sets brake mode, otherwise sets coast mode
   */
  public CommandBase setBrakeMode(boolean brake) {
    IdleMode sparkMode = brake ? IdleMode.kBrake : IdleMode.kCoast;
    NeutralMode victorMode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    return runOnce(() -> {
      m_leftLeadMotor.setIdleMode(sparkMode);
      m_leftFollowMotor.setNeutralMode(victorMode);
      m_rightLeadMotor.setIdleMode(sparkMode);
      m_rightFollowMotor.setNeutralMode(victorMode);
    });
  }

  /**
   * 
   * @param trajectory a PathPlanner Trajectory to follow
   * @return a PPRamseteCommand that follows the given PathPlanner Trajectory
   */
  public CommandBase followTrajectoryCommand(PathPlannerTrajectory trajectory) {
    return new PPRamseteCommand(
        trajectory,
        this::getPose,
        m_ramseteController,
        m_feedforward,
        m_kinematics,
        this::getWheelSpeeds,
        m_leftPIDController,
        m_rightPIDController,
        this::tankDriveVolts,
        true,
        this // Requires this drive subsystem
    );
  }
}
