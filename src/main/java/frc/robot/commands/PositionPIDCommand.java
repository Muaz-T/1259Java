package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;

// Keep your project's classes / packages for these imports - adjust package names if needed:
      // (assumes DriveConstants.c_HolomonicTranslateP etc. exist)
import frc.robot.subsystems.DriveSubsystem;     // (assumes method names below exist)
import frc.robot.ISubsystemAccess;              // (assumes subsystem access interface)

/**
 * Java translation of the C++ PositionPIDCommand.
 *
 * Notes:
 *  - lifecycle method names are the Java ones (initialize/execute/isFinished/end)
 *  - member variable names preserved from C++
 */
public class PositionPIDCommand extends Command {
  // Subsystem reference (kept name exactly)
  private final DriveSubsystem m_driveSubsystem;

  // PathPlanner trajectory state (public field 'pose' is available)
  private final PathPlannerTrajectoryState m_goalState = new PathPlannerTrajectoryState();

  // PathPlanner holonomic controller constructed with the same P constants as C++
  // (constructor PPHolonomicDriveController(PIDConstants, PIDConstants) exists in Java)
  private final PPHolonomicDriveController m_driveController =
      new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(3.0, 0.0, 0.0)
      );

  private final Timer m_timer = new Timer();
  private final BooleanLogEntry m_logStartCommand;

  // Tolerances (converted to Java units)
  private static final Rotation2d c_RotationTolerance = Rotation2d.fromDegrees(2.0); // 2 degrees
  private static final double c_PositionTolerance = Units.inchesToMeters(0.4);       // 0.4 in -> meters
  private static final double c_SpeedTolerance = Units.feetToMeters(0.021);          // 0.021 ft/s -> m/s

  public PositionPIDCommand(ISubsystemAccess subsystemAccess, Pose2d goalPose) {
    // Keep variable names the same
    m_driveSubsystem = subsystemAccess.GetDrive();

    // PathPlannerTrajectoryState has a public field `pose` in the Java API.
    m_goalState.pose = goalPose;

    // Command requirement: drive subsystem
    addRequirements(m_driveSubsystem);

    // Data log entry (Java DataLog / BooleanLogEntry exists).
    DataLog log = subsystemAccess.GetLogger();
    m_logStartCommand = new BooleanLogEntry(log, "/PositionPIDCommand/startCommand");
  }

  @Override
  public void initialize() {
    m_logStartCommand.append(true);
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    // PPHolonomicDriveController.calculateRobotRelativeSpeeds(...) returns ChassisSpeeds in Java
    ChassisSpeeds chassisSpeeds =
        m_driveController.calculateRobotRelativeSpeeds(m_driveSubsystem.GetPose(), m_goalState);

    // Drive the robot using the chassis speeds.
    // The C++ code passed a default-constructed DriveFeedforwards (zeros).
    // In Java PathPlanner has DriveFeedforwards.zeros(numModules). Replace 0 with your module count.
    //
    // e.g. DriveFeedforwards.zeros(Config.NUM_MODULES)
    m_driveSubsystem.Drive(chassisSpeeds, DriveFeedforwards.zeros(4)); // <-- set module count
  }

  @Override
  public boolean isFinished() {
    // Find the pose error: currentPose relative to goal pose (Pose2d.relativeTo exists in WPILib Java)
    Pose2d diff = m_driveSubsystem.GetPose().relativeTo(m_goalState.pose);

    // Rotation difference (radians)
    double turnDiff = diff.getRotation().getRadians();
    double turnTolerance = c_RotationTolerance.getRadians();

    // Use MathUtil.isNear with wrap-around boundaries for angles (min/max = -PI..PI)
    boolean rotationOk = MathUtil.isNear(0.0, turnDiff, turnTolerance, -Math.PI, Math.PI);

    // Position error: norm of the translation (Translation2d.getNorm() / getNorm() exists)
    double posNorm = diff.getTranslation().getNorm();
    boolean positionOk = posNorm < c_PositionTolerance;

    // Speed check: assumes GetSpeed() returns m/s (matching your C++ semantics)
    boolean speedOk = m_driveSubsystem.GetSpeed() < c_SpeedTolerance;

    // Matches the C++ logic: rotation && position && speed OR timer elapsed (0.075 s)
    return (rotationOk && positionOk && speedOk) || m_timer.hasElapsed(0.075);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.Stop();
    m_logStartCommand.append(false);
  }
}
