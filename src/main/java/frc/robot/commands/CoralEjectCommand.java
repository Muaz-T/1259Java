package frc.robot.commands; // <-- uncomment/change to your package

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;

// These imports assume your project structure/names; adjust as needed.
import frc.robot.ISubsystemAccess;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

// If these constants live elsewhere, import them instead.

public class CoralEjectCommand extends Command {
  // Keep field names the same as in C++
  private final CoralManipulatorSubsystem m_coralSubsystem;
  private final LEDSubsystem              m_ledSubsystem;
  private final BooleanLogEntry           m_logStartCommand;

  public CoralEjectCommand(ISubsystemAccess subsystemAccess) {
    // Mirrors: m_coralSubsystem(subsystemAccess.GetCoral()), m_ledSubsystem(subsystemAccess.GetLED())
    m_coralSubsystem = subsystemAccess.GetCoral();
    m_ledSubsystem   = subsystemAccess.GetLED();

    // Mirrors AddRequirements(&Coral, &LED)
    addRequirements(m_coralSubsystem, m_ledSubsystem); // varargs Subsystem... per API. :contentReference[oaicite:2]{index=2}

    // Mirrors: wpi::log::DataLog& log = subsystemAccess.GetLogger(); BooleanLogEntry(log, "/CoralEjectCommand/startCommand")
    DataLog log = subsystemAccess.GetLogger();
    m_logStartCommand = new BooleanLogEntry(log, "/CoralEjectCommand/startCommand"); // append(boolean) below. :contentReference[oaicite:3]{index=3}
  }

  @Override
  public void initialize() {
    // m_logStartCommand.Append(true);
    m_logStartCommand.append(true); // Java uses append(boolean). :contentReference[oaicite:4]{index=4}

    // m_coralSubsystem.EjectCoral(false);
    m_coralSubsystem.EjectCoral(false);

    // LED calls (unchanged names)
    m_ledSubsystem.SetCurrentAction(LEDSubsystem.ECurrentAction.kDefaultAction);
    m_ledSubsystem.SetAnimation(LEDSubsystem.c_defaultColor, LEDSubsystem.EAnimation.kSolid);
  }

  @Override
  public void execute() {
    // no-op, like the C++ Execute()
  }

  @Override
  public boolean isFinished() {
    // return m_coralSubsystem.IsCoralPresentOutput() == false;
    return m_coralSubsystem.IsCoralPresentOutput() == false;
  }

  @Override
  public void end(boolean interrupted) {
    // Mirrors C++ End()
    m_coralSubsystem.RetractManipulator();
    m_coralSubsystem.Stop();
    m_logStartCommand.append(false);
  }
}
