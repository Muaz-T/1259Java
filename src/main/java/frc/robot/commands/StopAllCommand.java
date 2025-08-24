package frc.robot.commands; // adjust to your package

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.ISubsystemAccess;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Java translation of StopAllCommand (C++).
 *
 * Member names and class name preserved:
 *   - m_intakeSubsystem
 *   - m_coralSubsystem
 *   - m_elevSubsystem
 *   - m_drive
 *   - m_vision
 *   - m_logStartCommand
 *
 * Lifecycle methods use Java names so scheduler will call them.
 */
public class StopAllCommand extends Command {

    private final IntakeSubsystem m_intakeSubsystem;
    private final CoralManipulatorSubsystem m_coralSubsystem;
    private final ElevatorSubsystem m_elevSubsystem;
    // private final ClimberSubsystem m_climbSubsystem; // commented out like C++ code
    private final DriveSubsystem m_drive;
    private final VisionSubsystem m_vision;

    private final BooleanLogEntry m_logStartCommand;

    public StopAllCommand(ISubsystemAccess subsystemAccess) {
        m_intakeSubsystem = subsystemAccess.GetIntake();
        m_coralSubsystem = subsystemAccess.GetCoral();
        m_elevSubsystem = subsystemAccess.GetElevator();
        // m_climbSubsystem = subsystemAccess.GetClimber();
        m_drive = subsystemAccess.GetDrive();
        m_vision = subsystemAccess.GetVision();

        // Add requirements for all subsystems
        addRequirements(
            m_intakeSubsystem,
            m_coralSubsystem,
            m_elevSubsystem,
            // m_climbSubsystem,
            m_drive,
            m_vision
        );

        // Create BooleanLogEntry for start/stop logging
        DataLog log = subsystemAccess.GetLogger();
        m_logStartCommand = new BooleanLogEntry(log, "/StopAllCommand/startCommand");
    }

    @Override
    public void initialize() {
        m_logStartCommand.append(true);

        // Cancel all currently scheduled commands
        CommandScheduler.getInstance().cancelAll();

        // Stop all subsystems
        m_intakeSubsystem.Stop();
        m_coralSubsystem.Stop();
        m_elevSubsystem.Stop();
        // m_climbSubsystem.Stop();
        m_drive.Stop();
        // Vision subsystem does not have Stop() in C++, so not called here
    }

    @Override
    public void execute() {
        // No-op, matches C++ behavior
    }

    @Override
    public boolean isFinished() {
        return true; // finishes immediately like C++
    }

    @Override
    public void end(boolean interrupted) {
        m_logStartCommand.append(false);
    }
}
