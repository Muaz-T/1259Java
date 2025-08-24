package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.ISubsystemAccess;
import frc.robot.Constants;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// If you have LED in your project, keep these imports:
import frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;

public class ClimbDeployCommand extends Command {
  // Keep variable names the same as C++
  private final ClimberSubsystem m_climbSubsystem;
  private final IntakeSubsystem m_intake;
  // If LED isn’t present in your build, you can remove this field and related code.
  private final LEDSubsystem m_led;

  private final BooleanLogEntry m_logStartCommand;

  public ClimbDeployCommand(ISubsystemAccess subsystemAccess) {
    // Grab subsystem singletons from your accessor (same pattern as C++).
    this.m_climbSubsystem = subsystemAccess.GetClimber();
    this.m_intake         = subsystemAccess.GetIntake();
    // If your ISubsystemAccess doesn’t provide LED, set m_led = null and remove LED calls below.
    this.m_led            = subsystemAccess.GetLED();

    // Declare requirements so the scheduler manages conflicts.
    addRequirements(m_climbSubsystem, m_intake, m_led);


    // DataLog is the Java equivalent of wpi::log::DataLog
    DataLog log = subsystemAccess.GetLogger();
    this.m_logStartCommand = new BooleanLogEntry(log, "/ClimbDeployCommand/startCommand");
  }

  @Override
  public void initialize() {
    m_logStartCommand.append(true);

    // Park intake before climbing (same call as C++)
    m_intake.ParkIntakeForClimb();

    // LED feedback (guarded in case LED isn’t wired into your build)
    if (m_led != null) {
      m_led.SetCurrentAction(LEDSubsystem.ECurrentAction.kClimbing);
      m_led.SetAnimation(LEDSubsystem.c_colorGreen, LEDSubsystem.EAnimation.kFlow);
    }

    // Start moving the climber to the deploy position
    m_climbSubsystem.GoToPosition(370.0);
  }

  @Override
  public void execute() {
    // No periodic action required (matches your C++)
  }

  @Override
  public boolean isFinished() {
    // Same tolerance and logic as C++
    return Math.abs(m_climbSubsystem.GetPosition() - 370.0) <= 20.0;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_led != null) {
      m_led.SetAnimation(LEDSubsystem.c_colorGreen, LEDSubsystem.EAnimation.kSolid);
      m_led.SetCurrentAction(LEDSubsystem.ECurrentAction.kClimbFinish);
    }
    m_logStartCommand.append(false);
  }
}
