package frc.robot.commands; // <- set to your package

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;

import frc.robot.ISubsystemAccess;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants.OperatorConstants.ELevels;

/**
 * Java port of ElevatorGoToCommand (LED present).
 * Field/method names kept identical to the C++ header.
 */
public class ElevatorGoToCommand extends Command {
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final LEDSubsystem m_ledSubsystem;

  private ELevels m_level = ELevels.L1;
  private final Timer m_timer = new Timer();
  private boolean m_bUsePresetLevel = false;

  private final BooleanLogEntry m_logStartCommand;

  // 2-arg convenience constructor to match C++ default parameter behavior
  public ElevatorGoToCommand(ISubsystemAccess subsystemAccess, ELevels eLevel) {
    this(subsystemAccess, eLevel, false);
  }

  public ElevatorGoToCommand(ISubsystemAccess subsystemAccess, ELevels eLevel, boolean bUsePresetLevel) {
    m_elevatorSubsystem = subsystemAccess.GetElevator();
    m_ledSubsystem = subsystemAccess.GetLED();

    m_level = eLevel;
    m_bUsePresetLevel = bUsePresetLevel;

    // AddRequirements(&GetElevator(), &GetLED()) -> addRequirements(...)
    addRequirements(m_elevatorSubsystem, m_ledSubsystem);

    DataLog log = subsystemAccess.GetLogger();
    m_logStartCommand = new BooleanLogEntry(log, "/ElevatorGoToCommand/startCommand");
  }

  @Override
  public void initialize() {
    m_logStartCommand.append(true);
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    if (m_bUsePresetLevel) {
      // same call as C++: GoToPresetLevel()
      m_elevatorSubsystem.GoToPresetLevel();
    } else {
      // same call as C++: GoToPosition(m_level)
      m_elevatorSubsystem.GoToPosition(m_level);
    }
  }

  @Override
  public boolean isFinished() {
    // C++ returned true unconditionally; keep identical behavior.
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_logStartCommand.append(false);
  }
}
