package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.util.datalog.DataLog;

public interface ISubsystemAccess {
    DriveSubsystem GetDrive();
    VisionSubsystem GetVision();
    ElevatorSubsystem GetElevator();
    CoralManipulatorSubsystem GetCoral();
    ClimberSubsystem GetClimber();
    LEDSubsystem GetLED(); // remove if you don’t have LED
    IntakeSubsystem GetIntake();
    DataLog GetLogger();
}
