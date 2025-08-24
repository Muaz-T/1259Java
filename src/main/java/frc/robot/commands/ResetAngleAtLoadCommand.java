package frc.robot.commands; // change to match your project package

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;

import frc.robot.ISubsystemAccess;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Java translation of ResetAngleAtLoadCommand (C++).
 *
 * Field names and class name preserved from original C++:
 *   - m_drive
 *   - m_logStartCommand
 *
 * Lifecycle methods use Java names (initialize/execute/isFinished/end) so they override CommandBase.
 */
public class ResetAngleAtLoadCommand extends Command {
  // kept exactly the same name as your C++ member
  private final DriveSubsystem m_drive;

  // kept exactly the same name as your C++ member
  private final BooleanLogEntry m_logStartCommand;

  public ResetAngleAtLoadCommand(ISubsystemAccess subsystemAccess) {
    // preserve exact member name
    m_drive = subsystemAccess.GetDrive();

    // register requirement (same effect as AddRequirements in C++)
    addRequirements(m_drive);

    // DataLog and BooleanLogEntry: get logger from subsystem access and create boolean entry
    DataLog log = subsystemAccess.GetLogger();
    m_logStartCommand = new BooleanLogEntry(log, "/ResetAngleAtLoadCommand/startCommand");
  }

  @Override
  public void initialize() {
    m_logStartCommand.append(true);

    // constants translated from C++:
    final Rotation2d c_loadStationBlueAngle = Rotation2d.fromDegrees(54.0);  // 54_deg
    final Rotation2d c_loadStationRedAngle = Rotation2d.fromDegrees(126.0);  // 126_deg
    final double c_centerLineY = Units.inchesToMeters(158.50);               // 158.50_in -> meters

    // DriverStation.getAlliance() returns Optional<DriverStation.Alliance> in Java.
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      // default to blue (matches the C++ flow)
      Rotation2d loadStationAngle = c_loadStationBlueAngle;

      // m_drive.GetY() preserved (assumes your DriveSubsystem Java has GetY())
      boolean bAboveCenterLine = (m_drive.GetY() > c_centerLineY);
      if (bAboveCenterLine) {
        // invert angle (same effect as unary minus in C++)
        loadStationAngle = Rotation2d.fromRadians(-loadStationAngle.getRadians());
      }

      // check for red alliance; DriverStation.Alliance enum values are Red and Blue
      if (alliance.get() == DriverStation.Alliance.Red) {
        loadStationAngle = c_loadStationRedAngle;
        if (bAboveCenterLine) {
          loadStationAngle = Rotation2d.fromRadians(-loadStationAngle.getRadians());
        }
      }

      System.out.printf("Resetting pose angle at load station to %.3f\n", loadStationAngle.getDegrees());
      // Build a Pose2d in meters (Pose2d(double xMeters, double yMeters, Rotation2d rotation))
      Pose2d resetPose = new Pose2d(m_drive.GetX(), m_drive.GetY(), loadStationAngle);
      m_drive.ResetOdometry(resetPose);
    } else {
      System.out.println("Could not determine alliance - skipping reset pose angle at load station");
    }
  }

  @Override
  public void execute() {
    // no-op (same as the C++ code)
  }

  @Override
  public boolean isFinished() {
    // command finishes immediately (same as C++: return true)
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_logStartCommand.append(false);
  }
}
