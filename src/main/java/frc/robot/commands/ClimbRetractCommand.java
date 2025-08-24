// Adjust the package to match your project structure
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command; // Command lifecycle: initialize/execute/isFinished/end
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;

import frc.robot.ISubsystemAccess;         // Your team interface
import frc.robot.subsystems.ClimberSubsystem; // Your team subsystem
import frc.robot.subsystems.LEDSubsystem;     // Your team subsystem (remove if not used)

public class ClimbRetractCommand extends Command {

  // --- Fields (kept same names as C++) ---
  private final ClimberSubsystem m_climbSubsystem;
  private final LEDSubsystem m_led; // If you don't have LED, delete this + related lines
  private BooleanLogEntry m_logStartCommand;

  // --- Constructor (kept same name/signature) ---
  public ClimbRetractCommand(ISubsystemAccess subsystemAccess) {
    // Equivalent to member init list
    this.m_climbSubsystem = subsystemAccess.GetClimber();
    this.m_led = subsystemAccess.GetLED(); // Remove if not using LED

    // AddRequirements({...}) → addRequirements(varargs)
    addRequirements(this.m_climbSubsystem, this.m_led); // If no LED, just pass m_climbSubsystem

    // wpi::log::DataLog & BooleanLogEntry → edu.wpi.first.util.datalog.*
    DataLog log = subsystemAccess.GetLogger();
    this.m_logStartCommand = new BooleanLogEntry(log, "/ClimbRetractCommand/startCommand");
  }

  // --- Command lifecycle methods ---
  @Override
  public void initialize() {
    m_logStartCommand.append(true);

    // LED guarded by #ifdef in C++; keep the behavior (delete if you don't have LED)
    m_led.SetAnimation(LEDSubsystem.c_colorBlue, LEDSubsystem.EAnimation.kFlow);

    // Same call/constant names as your codebase
    m_climbSubsystem.GoToPosition(-20.0);
  }

  @Override
  public void execute() {
    // No-op (intentional)
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_climbSubsystem.GetPosition() - -20.0) <= 20.0;
  }

  @Override
  public void end(boolean interrupted) {
    // LED behavior at end (delete if not using LED)
    m_led.SetAnimation(LEDSubsystem.c_colorBlue, LEDSubsystem.EAnimation.kSolid);
    m_led.SetCurrentAction(LEDSubsystem.ECurrentAction.kClimbFinish);

    m_logStartCommand.append(false);
  }
}
