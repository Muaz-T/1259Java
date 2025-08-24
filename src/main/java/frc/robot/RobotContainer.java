// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//___________________________________________________________________________________________________________________________________________________________________________________
// HEADER FILE IMPORTS

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.ISubsystemAccess;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
//___________________________________________________________________________________________________________________________________________________________________________________


//___________________________________________________________________________________________________________________________________________________________________________________
// CPP FILE IMPORTS
import frc.robot.commands.ElevatorGoToCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralPrepCommand;
import frc.robot.commands.CoralEjectCommand;
import frc.robot.commands.CoralEjectPostCommand;
import frc.robot.commands.StopAllCommand;
import frc.robot.commands.ClimbRetractCommand;
import frc.robot.commands.ClimbDeployCommand;
import frc.robot.commands.PositionPIDCommand;
import frc.robot.commands.ResetAngleAtLoadCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
//___________________________________________________________________________________________________________________________________________________________________________________


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements ISubsystemAccess {

  //____________________________________________________________________________________________________________________
  // HEADER FILE
  enum ESideSelected{
    Unselected,
    LeftSide,
    RightSide
  };

  @Override public DriveSubsystem GetDrive() {return m_drive; }
  @Override public VisionSubsystem GetVision() {return m_vision; }
  @Override public ElevatorSubsystem GetElevator() {return m_elevator; }
  @Override public CoralManipulatorSubsystem GetCoral() {return m_coral; }
  @Override public IntakeSubsystem GetIntake() {return m_intake; }
  @Override public ClimberSubsystem GetClimber() {return m_climber; }
  @Override public LEDSubsystem GetLED() {return m_led; }

  @Override public DataLog GetLogger() { return DataLogManager.getLog(); };

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public void StartUp(){
    m_intake.AlignIntake();
    m_timer.reset();
    m_timer.start();
    m_climber.GoToPosition(100.0);
  }

  void SetHighSpeed(){m_drive.SetSlowSpeed(false);}

  public static RobotContainer m_pThis;

  DriveSubsystem m_drive;
  VisionSubsystem m_vision;
  ElevatorSubsystem m_elevator;
  CoralManipulatorSubsystem m_coral;
  IntakeSubsystem m_intake;
  ClimberSubsystem m_climber;
  LEDSubsystem m_led;

  CommandXboxController m_primaryController = new CommandXboxController(0);
  CommandXboxController m_secondaryController = new CommandXboxController(1);

  CommandXboxController m_buttonController = new CommandXboxController(3);

  boolean m_fieldRelative = true;
  boolean m_isAutoRunning = false;

  ESideSelected m_sideSelected = ESideSelected.Unselected;

// Toggle drive modes
private final InstantCommand m_toggleFieldRelative =
    new InstantCommand(() -> m_fieldRelative = !m_fieldRelative);

private final InstantCommand m_toggleSlowSpeed =
    new InstantCommand(() -> m_drive.ToggleSlowSpeed(), m_drive);

private final InstantCommand m_setHighSpeedCmd =
    new InstantCommand(() -> m_drive.SetSlowSpeed(false), m_drive);

// Elevator presets
private final InstantCommand m_elevL4 =
    new InstantCommand(() -> m_elevator.GoToPosition(Constants.OperatorConstants.ELevels.L4), m_elevator);

private final InstantCommand m_elevL3 =
    new InstantCommand(() -> m_elevator.GoToPosition(Constants.OperatorConstants.ELevels.L3), m_elevator);

private final InstantCommand m_elevL2 =
    new InstantCommand(() -> m_elevator.GoToPosition(Constants.OperatorConstants.ELevels.L2), m_elevator);

private final InstantCommand m_elevL1 =
    new InstantCommand(() -> m_elevator.GoToPosition(0.0), m_elevator);

private final InstantCommand m_elevL2_3 =
    new InstantCommand(() -> m_elevator.GoToPosition(Constants.OperatorConstants.ELevels.algaeRemovalL2_3), m_elevator);

private final InstantCommand m_elevL3_4 =
    new InstantCommand(() -> m_elevator.GoToPosition(Constants.OperatorConstants.ELevels.algaeRemovalL3_4), m_elevator);

// Preset levels
private final InstantCommand m_setL1 =
    new InstantCommand(() -> m_elevator.SetPresetLevel(Constants.OperatorConstants.ELevels.L1), m_elevator);

private final InstantCommand m_setL2 =
    new InstantCommand(() -> m_elevator.SetPresetLevel(Constants.OperatorConstants.ELevels.L2), m_elevator);

private final InstantCommand m_setL3 =
    new InstantCommand(() -> m_elevator.SetPresetLevel(Constants.OperatorConstants.ELevels.L3), m_elevator);

private final InstantCommand m_setL4 =
    new InstantCommand(() -> m_elevator.SetPresetLevel(Constants.OperatorConstants.ELevels.L4), m_elevator);

// Side select
private final InstantCommand m_setRight =
    new InstantCommand(() -> SetSideSelected(ESideSelected.RightSide));

private final InstantCommand m_setLeft =
    new InstantCommand(() -> SetSideSelected(ESideSelected.LeftSide));

// Elevator controls
private final InstantCommand m_elevReset =
    new InstantCommand(() -> m_elevator.ElevatorReset(), m_elevator);

private final InstantCommand m_elevRelPosUp =
    new InstantCommand(() -> m_elevator.GotoPositionRel(1.0), m_elevator);

private final InstantCommand m_elevRelPosDown =
    new InstantCommand(() -> m_elevator.GotoPositionRel(-1.0), m_elevator);

// Intake
private final InstantCommand m_intakeAlign =
    new InstantCommand(() -> m_intake.ParkIntakeForLoad(), m_intake);

// Coral
private final InstantCommand m_coralStop =
    new InstantCommand(() -> m_coral.Stop(), m_coral);

private final InstantCommand m_coralRetract =
    new InstantCommand(() -> m_coral.RetractCoral(Constants.OperatorConstants.ELevels.L1), m_coral);

private final InstantCommand m_coralDeployManip =
    new InstantCommand(() -> m_coral.DeployManipulator(), m_coral);

private final InstantCommand m_coralDeployManipAlgae =
    new InstantCommand(() -> m_coral.DeployManipulatorAlgae(), m_coral);

private final InstantCommand m_coralRetractManip =
    new InstantCommand(() -> m_coral.RetractManipulator(), m_coral);

private final InstantCommand m_coralAlgaeRemove =
    new InstantCommand(() -> m_coral.SetManipulator(0.9), m_coral);

// Climber
private final InstantCommand m_climberDeployRelUp =
    new InstantCommand(() -> m_climber.GoToPositionRel(20.0), m_climber);

private final InstantCommand m_climberDeployRelDown =
    new InstantCommand(() -> m_climber.GoToPositionRel(-20.0), m_climber);

// Drive
private final InstantCommand m_driveStop =
    new InstantCommand(() -> m_drive.DriveBack(), m_drive);

// LEDs
private final InstantCommand m_followPathLED =
    new InstantCommand(() -> {
        m_led.SetCurrentAction(LEDSubsystem.ECurrentAction.kFollowPath);
        m_led.SetAnimation(LEDSubsystem.c_colorPurple, LEDSubsystem.EAnimation.kScanner);
    }, m_led);

private final InstantCommand m_endLED =
    new InstantCommand(() -> m_led.SetCurrentAction(LEDSubsystem.ECurrentAction.kIdle), m_led);

Timer m_timer;

  //_____________________________________________________________________________________________________________________





  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_pThis = this;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
