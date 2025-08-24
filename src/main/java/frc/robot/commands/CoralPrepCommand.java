package frc.robot.commands; // <-- set to your package

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;

import frc.robot.ISubsystemAccess;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants.OperatorConstants.ELevels;

/**
 * Java translation of CoralPrepCommand (LED present).
 * Field & method names kept the same as your C++ header.
 */
public class CoralPrepCommand extends Command {
  // same fields as C++ header
  private final CoralManipulatorSubsystem m_coralSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final LEDSubsystem m_ledSubsystem;

  private final Timer m_timer = new Timer();

  // In C++ header: ELevels m_coralLevel = L1;
  // Use explicit enum qualification in Java (assumes ELevels enum exists in your codebase).
  private ELevels m_coralLevel = ELevels.L1;
  private double m_coralEncPos = 0.0;
  private boolean m_retract = true;
  private boolean m_blocked = false;

  private final BooleanLogEntry m_logStartCommand;

  public CoralPrepCommand(ISubsystemAccess subsystemAccess) {
    // mirror member initialization list from C++
    m_coralSubsystem = subsystemAccess.GetCoral();
    m_elevatorSubsystem = subsystemAccess.GetElevator();
    m_ledSubsystem = subsystemAccess.GetLED();

    // AddRequirements(&Coral, &Elevator, &LED) -> addRequirements(...)
    addRequirements(m_coralSubsystem, m_elevatorSubsystem, m_ledSubsystem);

    // DataLog / BooleanLogEntry construction mirrors C++ usage
    DataLog log = subsystemAccess.GetLogger();
    m_logStartCommand = new BooleanLogEntry(log, "/CoralPrepCommand/startCommand");
  }

  @Override
  public void initialize() {
    m_logStartCommand.append(true);

    // Read the SmartDashboard key exactly like the C++ code
    double turns = SmartDashboard.getNumber("CoralRetractTurns", 3.25);

    m_timer.reset();
    m_timer.start();

    m_coralLevel = m_elevatorSubsystem.GetPresetLevel();
    m_blocked = (m_coralLevel == ELevels.L4 || m_coralLevel == ELevels.L3)
                && m_coralSubsystem.IsCoralPresentInput();
    if (m_blocked) {
      return;
    }

    // LED behavior (was under #ifdef LED in C++)
    m_ledSubsystem.SetCurrentAction(LEDSubsystem.ECurrentAction.kSequenceStart);
    m_ledSubsystem.SetAnimation(LEDSubsystem.c_colorBlue, LEDSubsystem.EAnimation.kFlow); // TODO: replace constants if needed

    if (m_coralLevel == ELevels.L4) {
      // anti-slam move
      m_elevatorSubsystem.GoToPosition(37.0);
    } else {
      m_elevatorSubsystem.GoToPresetLevel();
    }

    // refresh preset and set coral encoder target
    m_coralLevel = m_elevatorSubsystem.GetPresetLevel();
    m_coralEncPos = m_coralSubsystem.GetPosition() + turns;
    m_retract = true;
  }

  @Override
  public void execute() {
    if (m_retract) {
      m_coralSubsystem.RetractCoral(m_coralLevel);
      m_retract = false;
    }
  }

  @Override
  public boolean isFinished() {
    // return m_blocked || fabs(m_coralSubsystem.GetPosition() - m_coralEncPos) <= 1.0 || m_timer.HasElapsed(2.0_s);
    return m_blocked
        || Math.abs(m_coralSubsystem.GetPosition() - m_coralEncPos) <= 1.0
        || m_timer.hasElapsed(2.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_coralSubsystem.Stop();
    m_logStartCommand.append(false);
  }
}
