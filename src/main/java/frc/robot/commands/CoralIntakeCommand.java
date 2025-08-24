package frc.robot.commands; // <-- adjust to your package

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;

import frc.robot.ISubsystemAccess;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Java port of CoralIntakeCommand (LED enabled build).
 * Field & subsystem method names kept identical to the C++ version.
 */
public class CoralIntakeCommand extends Command {
  // fields kept exactly as the C++ header
  private final CoralManipulatorSubsystem m_coralSubsystem;
  private final LEDSubsystem m_ledSubsystem;
  private final BooleanLogEntry m_logStartCommand;

  public CoralIntakeCommand(ISubsystemAccess subsystemAccess) {
    // mirror C++ member init
    m_coralSubsystem = subsystemAccess.GetCoral();
    m_ledSubsystem = subsystemAccess.GetLED();

    // AddRequirements(&Coral, &LED) -> addRequirements(...) (varargs). See docs. :contentReference[oaicite:2]{index=2}
    addRequirements(m_coralSubsystem, m_ledSubsystem);

    // DataLog / BooleanLogEntry creation matches the C++ usage.
    DataLog log = subsystemAccess.GetLogger();
    m_logStartCommand = new BooleanLogEntry(log, "/CoralIntakeCommand/startCommand"); // append(boolean) used below. :contentReference[oaicite:3]{index=3}
  }

  @Override
  public void initialize() {
    // m_logStartCommand.Append(true);
    m_logStartCommand.append(true);

    // m_coralSubsystem.SetManipulator(-0.5);
    m_coralSubsystem.SetManipulator(-0.5);

    // #ifdef LED path in C++ -> direct call here for LED build
    m_ledSubsystem.SetCurrentAction(LEDSubsystem.ECurrentAction.kHasCoral);
  }

  @Override
  public void execute() {
    // if (m_coralSubsystem.IsCoralPresentOutput()) { ... }
    if (m_coralSubsystem.IsCoralPresentOutput()) {
      // LED call preserved exactly
      m_ledSubsystem.SetAnimation(LEDSubsystem.c_colorPink, LEDSubsystem.EAnimation.kSolid);
    }
  }

  @Override
  public boolean isFinished() {
    // return m_coralSubsystem.IsCoralPresentOutput() == true && m_coralSubsystem.IsCoralPresentInput() == false;
    return m_coralSubsystem.IsCoralPresentOutput() == true
        && m_coralSubsystem.IsCoralPresentInput() == false;
  }

  @Override
  public void end(boolean interrupted) {
    m_coralSubsystem.Stop();
    m_logStartCommand.append(false);
  }
}

