package frc.robot.commands; // adjust to your package

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.ISubsystemAccess;
import frc.robot.Constants.OperatorConstants.ELevels;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Java translation of CoralEjectPostCommand (LED present).
 * Keeps field names and command logic identical to the C++ original.
 */
public class CoralEjectPostCommand extends Command {
  // private fields kept exactly as in the C++ header
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final LEDSubsystem m_ledSubsystem;
  private final Timer m_timer = new Timer();
  private final BooleanLogEntry m_logStartCommand;

  // keep the same named constants referenced by your code (L1 etc.)
  // public static final double L1 = ...; // leave as-is, assuming defined elsewhere

  public CoralEjectPostCommand(ISubsystemAccess subsystemAccess) {
    // mirror member initialization
    m_elevatorSubsystem = subsystemAccess.GetElevator();
    m_driveSubsystem = subsystemAccess.GetDrive();
    m_ledSubsystem = subsystemAccess.GetLED();

    // AddRequirements(&Elevator, &Drive, &LED) in C++ → addRequirements(...) in Java
    addRequirements(m_elevatorSubsystem, m_driveSubsystem, m_ledSubsystem);

    // DataLog and BooleanLogEntry construction matches C++ behavior
    DataLog log = subsystemAccess.GetLogger();
    m_logStartCommand = new BooleanLogEntry(log, "/CoralEjectPostCommand/startCommand");
  }

  @Override
  public void initialize() {
    m_logStartCommand.append(true);
    m_timer.reset();
    m_timer.start();

    // commented in the original C++ -- preserved comment
    // if (DriverStation.isTeleopEnabled()) {
    //     m_driveSubsystem.DriveBack();
    // }
  }

  @Override
  public void execute() {
    // if (m_timer.HasElapsed(0.25_s)) { m_elevatorSubsystem.GoToPosition(L1); }
    if (m_timer.hasElapsed(0.25)) {
      m_elevatorSubsystem.GoToPosition(ELevels.L1);
    }
  }

  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomousEnabled()) {
      return m_elevatorSubsystem.GetLowerLimit() || m_timer.hasElapsed(0.25);
    } else {
      return m_elevatorSubsystem.GetLowerLimit() || m_timer.hasElapsed(0.75);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Don't need to do this, the limit switch should turn off the motor (comment matched original)
    if (DriverStation.isTeleopEnabled()) {
      m_driveSubsystem.Stop();
    }
    m_logStartCommand.append(false);
  }
}
