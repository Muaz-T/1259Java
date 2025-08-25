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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import com.pathplanner.lib.path.PathPlannerPath;
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
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.HashMap;
import edu.wpi.first.math.util.Units;
import java.util.List;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

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

  public void SetHighSpeed(){m_drive.SetSlowSpeed(false);}

  public static Command getFollowPathCommand() {
    return m_pThis.GetFollowPathCommandImpl();
  }

  public static RobotContainer m_pThis = null;

  DriveSubsystem m_drive = new DriveSubsystem();
  VisionSubsystem m_vision = new VisionSubsystem();
  ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  CoralManipulatorSubsystem m_coral = new CoralManipulatorSubsystem();
  IntakeSubsystem m_intake = new IntakeSubsystem();
  ClimberSubsystem m_climber = new ClimberSubsystem();
  LEDSubsystem m_led = new LEDSubsystem();

  CommandXboxController m_primaryController = new CommandXboxController(0);
  CommandXboxController m_secondaryController = new CommandXboxController(1);

  CommandXboxController m_buttonController = new CommandXboxController(3);

  boolean m_fieldRelative = true;
  boolean m_isAutoRunning = false;

  
    DoubleLogEntry m_logRobotPoseX;
    DoubleLogEntry m_logRobotPoseY;
	DoubleLogEntry m_logRobotPoseRot;

	DoubleLogEntry m_logTargetPoseX;
	DoubleLogEntry m_logTargetPoseY;
	DoubleLogEntry m_logTargetPoseRot;

  ESideSelected m_sideSelected = ESideSelected.Unselected;

// Toggle drive modes
private final InstantCommand m_toggleFieldRelative =
    new InstantCommand(() -> m_fieldRelative = !m_fieldRelative);

private final InstantCommand m_toggleSlowSpeed =
    new InstantCommand(() -> m_drive.ToggleSlowSpeed(), m_drive);

private final InstantCommand m_setHighSpeedCmd =
    new InstantCommand(() -> m_drive.SetSlowSpeed(false), m_drive);

private final InstantCommand m_resetOdo =
    new InstantCommand(() -> {
        if (IsTagBeingImaged()){
        Pose2d tagPose = GetTagPose();
        Pose2d resetPose = new Pose2d(m_drive.GetX(), m_drive.GetY(), Rotation2d.fromDegrees(tagPose.getRotation().getDegrees()));
        m_drive.ResetOdometry(resetPose);
        }
    }, m_drive);

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

PathConstraints m_pathConstraints = new PathConstraints(1.5, 3.0, 180, 360);
PathPlannerPath m_path;

Timer m_timer = new Timer();
Pose2d m_targetPose;

  //_____________________________________________________________________________________________________________________

// CPP FILE

Field2d m_field = new Field2d();

double c_coralDeployDelay = 0.95;
double c_coralPostEjectDelay = 0.25;

public static class TagInfo
{
  public TagInfo(double targetRot
  , double leftX
  , double leftY
  , double midX
  , double midY
  , double rightX
  , double rightY){
    m_poseLeft = new Pose2d(leftX, leftY, Rotation2d.fromDegrees(targetRot));
    m_poseMid = new Pose2d(midX, midY, Rotation2d.fromDegrees(targetRot));
    m_poseRight = new Pose2d(rightX, rightY, Rotation2d.fromDegrees(targetRot));
  }

  public Pose2d m_poseLeft;
  public Pose2d m_poseMid;
  public Pose2d m_poseRight;

  public Pose2d GetPose(ESideSelected side) {
    switch (side) {
        case LeftSide:
            return m_poseLeft;
        case RightSide:
            return m_poseRight;
        default:
            return m_poseMid;
    }
}

};

public static final Map<Integer, Pose2d> c_mapTagLocations = new HashMap<>(Map.ofEntries(
  Map.entry(6,  new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300))),
  Map.entry(7,  new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0))),
  Map.entry(8,  new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60))),
  Map.entry(9,  new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120))),
  Map.entry(10, new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180))),
  Map.entry(11, new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240))),
  Map.entry(17, new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240))),
  Map.entry(18, new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180))),
  Map.entry(19, new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120))),
  Map.entry(20, new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60))),
  Map.entry(21, new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0))),
  Map.entry(22, new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300)))
));

public static final Map<Integer, TagInfo> c_mapTagPoses = new HashMap<>(Map.ofEntries(
  Map.entry(6, new TagInfo(
      120, 535.86, 107.87, 541.49, 111.12, 547.12, 114.37
  )),
  Map.entry(7, new TagInfo(
      180, 568.87, 152.00, 568.87, 158.50, 568.87, 165.00
  )),
  Map.entry(8, new TagInfo(
      240, 547.12, 202.63, 541.49, 205.88, 535.86, 209.13
  )),
  Map.entry(9, new TagInfo(
      300, 492.40, 209.13, 486.77, 205.88, 481.14, 202.63
  )),
  Map.entry(10, new TagInfo(
      0, 453.89, 165.00, 453.80, 158.50, 453.89, 152.00
  )),
  Map.entry(11, new TagInfo(
      60, 481.14, 114.37, 484.27, 106.79, 492.40, 107.87
  )),
  Map.entry(17, new TagInfo(
      60, 143.76, 114.37, 149.39, 111.12, 155.02, 107.87
  )),
  Map.entry(18, new TagInfo(
      0, 122.00, 165.00, 122.00, 158.50, 122.00, 152.00
  )),
  Map.entry(19, new TagInfo(
      300, 155.02, 209.13, 149.39, 205.88, 143.76, 202.63
  )),
  Map.entry(20, new TagInfo(
      240, 209.73, 202.63, 204.10, 205.88, 198.47, 209.13
  )),
  Map.entry(21, new TagInfo(
      180, 231.49, 152.00, 231.49, 158.50, 231.49, 165.00
  )),
  Map.entry(22, new TagInfo(
      120, 198.47, 107.87, 204.10, 111.12, 209.73, 114.37
  ))
));

public RobotContainer() {
  NamedCommands.registerCommand("RaiseL4",
  new SequentialCommandGroup(m_setL4, m_elevL4));

NamedCommands.registerCommand("RaiseL3",
  new SequentialCommandGroup(m_setL3, m_elevL3));

NamedCommands.registerCommand("RaiseL2",
  new SequentialCommandGroup(m_setL2, m_elevL2));

NamedCommands.registerCommand("PlaceL2",
  new SequentialCommandGroup(
      m_setL2,
      new CoralPrepCommand(this),
      new CoralEjectCommand(this),
      m_elevL1
  ));

NamedCommands.registerCommand("ShootCoral",
  new SequentialCommandGroup(
      new CoralEjectCommand(this),
      m_elevL1
  ));

NamedCommands.registerCommand("StopCoral",
  new SequentialCommandGroup(m_coralStop));

NamedCommands.registerCommand("PlaceL4",
  new SequentialCommandGroup(
      m_setL4,
      m_setRight,
      new CoralPrepCommand(this),
      m_setHighSpeedCmd,
      new ConditionalCommand(
          new SequentialCommandGroup(
              new ElevatorGoToCommand(this, Constants.OperatorConstants.ELevels.L4, true),
              new InstantCommand(() -> m_coral.DeployManipulator(), m_coral)
          ),
          new InstantCommand(() -> m_coral.RetractManipulator(), m_coral),
          () -> m_elevator.GetPresetLevel() == Constants.OperatorConstants.ELevels.L4 || m_elevator.GetPresetLevel() == Constants.OperatorConstants.ELevels.L1
      ),
      new WaitCommand(c_coralDeployDelay),
      new CoralEjectCommand(this),
      new WaitCommand(c_coralPostEjectDelay),
      m_elevL1
  ));

NamedCommands.registerCommand("Intake",
  new SequentialCommandGroup(
      m_elevL1,
      new CoralIntakeCommand(this),
      m_elevL3
  ));

  DataLog log = GetLogger();

SmartDashboard.putNumber("InitPose", 180.0);
SmartDashboard.putData("Field", m_field);

m_logRobotPoseX = new DoubleLogEntry(log, "/path/robotX");
m_logRobotPoseY = new DoubleLogEntry(log, "/path/robotY");
m_logRobotPoseRot = new DoubleLogEntry(log, "/path/robotRot");

m_logTargetPoseX = new DoubleLogEntry(log, "/path/targetX");
m_logTargetPoseY = new DoubleLogEntry(log, "/path/targetY");
m_logTargetPoseRot = new DoubleLogEntry(log, "/path/targetRot");

  // Logging callback for current robot pose
PathPlannerLogging.setLogCurrentPoseCallback((Pose2d pose) -> {
  // In Java, Pose2d#getX / #getY return meters as doubles, and #getRotation().getDegrees() returns degrees
  m_logRobotPoseX.append(pose.getX());
  m_logRobotPoseY.append(pose.getY());
  m_logRobotPoseRot.append(pose.getRotation().getDegrees());

  m_field.setRobotPose(pose);
});

// Logging callback for target robot pose
PathPlannerLogging.setLogTargetPoseCallback((Pose2d pose) -> {
  m_logTargetPoseX.append(pose.getX());
  m_logTargetPoseY.append(pose.getY());
  m_logTargetPoseRot.append(pose.getRotation().getDegrees());

  m_field.getObject("target pose").setPose(pose);
});

// Logging callback for the active path, sent as a list of poses
PathPlannerLogging.setLogActivePathCallback((List<Pose2d> poses) -> {
  m_field.getObject("path").setPoses(poses);
});

  m_pThis = this;

  if (!AutoBuilder.isConfigured()) {
    AutoBuilder.configure(
        () -> m_drive.GetPose(),                  // Supplier<Pose2d>
        (pose) -> m_drive.ResetOdometry(pose),    // Consumer<Pose2d>
        () -> m_drive.GetChassisSpeeds(),         // Supplier<ChassisSpeeds>
        (ChassisSpeeds speeds) -> {
          // Note: negative signs preserved from C++
          m_drive.Drive(-speeds.vxMetersPerSecond, 
                        -speeds.vyMetersPerSecond, 
                        -speeds.omegaRadiansPerSecond, 
                        false);
        },
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.2),
            new PIDConstants(3.0, 0.0, 0.2)
        ),
        m_drive.GetRobotCfg(),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        m_drive
    );
  }

  m_chooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("Auto", m_chooser);

  SetDefaultComamands();
  configureBindings();

  CalcTargetPoses();
}

public void CalcTargetPoses(){
    double c_reefOffset = 0.635;
    double c_leftRightOffset = 0.1651;

    for (Map.Entry<Integer, Pose2d> tagLoc : c_mapTagLocations.entrySet()) {
        // int tagId = tagLoc.first;  auto& tagPose = tagLoc.second;
        int tagId = tagLoc.getKey();
        Pose2d tagPose = tagLoc.getValue();
  
        // auto thetaDeg = fmod((tagPose.Rotation().Degrees().value() + 90.0), 360.0);
        double thetaDeg = normalizeDegrees(tagPose.getRotation().getDegrees() + 90.0);
  
        // double thetaRads = thetaDeg * std::numbers::pi / 180.0;
        double thetaRads = Math.toRadians(thetaDeg);
  
        // double midX = tagPose.X().value() + c_reefOffset.value() * sin(thetaRads);
        // double midY = tagPose.Y().value() - c_reefOffset.value() * cos(thetaRads);
        double midX = tagPose.getX() + c_reefOffset * Math.sin(thetaRads);
        double midY = tagPose.getY() - c_reefOffset * Math.cos(thetaRads);
  
        // double targetRot = fmod((tagPose.Rotation().Degrees().value() + 180.0), 360.0);
        double targetRot = normalizeDegrees(tagPose.getRotation().getDegrees() + 180.0);
  
        // double leftRightOffsetX = c_leftRightOffset.value() * cos(thetaRads);
        // double leftRightOffsetY = -c_leftRightOffset.value() * sin(thetaRads);
        double leftRightOffsetX = c_leftRightOffset * Math.cos(thetaRads);
        double leftRightOffsetY = -c_leftRightOffset * Math.sin(thetaRads);
  
        // double leftX = midX - leftRightOffsetX;  double leftY = midY + leftRightOffsetY;
        double leftX = midX - leftRightOffsetX;
        double leftY = midY + leftRightOffsetY;
  
        // double rightX = midX + leftRightOffsetX; double rightY = midY - leftRightOffsetY;
        double rightX = midX + leftRightOffsetX;
        double rightY = midY - leftRightOffsetY;
  
        // TagInfo ti(targetRot, leftX, leftY, midX, midY, rightX, rightY);
        // c_mapTagPoses.emplace(std::make_pair(tagId, ti));
        TagInfo ti = new TagInfo(targetRot, leftX, leftY, midX, midY, rightX, rightY);
        c_mapTagPoses.putIfAbsent(tagId, ti); // putIfAbsent(...) if you want C++ emplace behavior
      }
}

private static double normalizeDegrees(double deg) {
    double d = deg % 360.0;
    return d < 0 ? d + 360.0 : d;
}

public void Periodic(){
    if (m_timer.isRunning() && m_timer.hasElapsed(0.75)){
        m_intake.ParkIntakeForLoad();
        m_timer.stop();
    }
    
    int count = 0;

    Pose2d targetPose = new Pose2d(10.252, 3.55, Rotation2d.fromDegrees(90));
    double pathLen = 0.0;
    if (false){

    }
    else if (count % 25 == 0){
        ConfigureRobotLEDs();
    }

    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
    m_field.setRobotPose(m_drive.GetPose());
    SmartDashboard.putData("Field", m_field);

    AreWeInTheSweetSpot();
}

public void AreWeInTheSweetSpot(){
    Pose2d targetPose;
    if (IsTagBeingImaged()){
        targetPose = GetTagPose();

        var targetX = targetPose.getX();
        var targetY = targetPose.getY();
        var targetRot = targetPose.getRotation().getDegrees();
  
    SmartDashboard.putNumber("targetX", targetX);
    SmartDashboard.putNumber("targetY", targetY);
    SmartDashboard.putNumber("targetRot", targetRot);

    double pathLen = m_drive.GetCurrentPose().getTranslation().getDistance(targetPose.getTranslation());

    if (pathLen > 0.6 && pathLen < 1.00)
    {
        m_led.SetCurrentAction(LEDSubsystem.ECurrentAction.kTagVisible);
        m_led.SetAnimation(LEDSubsystem.c_colorWhite, LEDSubsystem.EAnimation.kStrobe);
    }
    else if ((m_led.GetCurrentAction() == LEDSubsystem.ECurrentAction.kTagVisible)){
        m_led.SetCurrentAction(LEDSubsystem.ECurrentAction.kIdle);
    }
  }
}

public Pose2d GetTagPose() {
    int tagId = m_vision.GetTagId();
    TagInfo ti = c_mapTagPoses.get(tagId);
    if (ti != null) {
        return ti.GetPose(m_sideSelected);
    }
    return null; // no tag
}

public boolean IsTagBeingImaged() {
    if (m_vision.GetTagId() != -1){
        return true;
    }
    return false;
}

public void SetDefaultComamands(){
    m_drive.setDefaultCommand(
        new RunCommand(() -> {
            // Don't send any input if autonomous is running
            if (!m_isAutoRunning) {

                final double kDeadband = 0.02;
                final double direction = 1.0;

                double xInput = direction * m_primaryController.getLeftY();
                double yInput = direction * m_primaryController.getLeftX();
                double rotInput = m_primaryController.getRightX();

                double xSpeed = xInput * m_drive.m_currentMaxSpeed;
                double ySpeed = yInput * m_drive.m_currentMaxSpeed;
                double rot = rotInput * m_drive.m_currentMaxAngularSpeed;

                // Send speeds to the drive subsystem
                GetDrive().Drive(xSpeed, ySpeed, rot, m_fieldRelative);
            }
        }, m_drive) // Dependencies: requires m_drive
    );
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


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    ConfigPrimaryButtonBindings();
    ConfigButtonBoxBindings();
  }

public void ConfigPrimaryButtonBindings(){
    var primary = m_primaryController;

    primary.a().onTrue(
        new SequentialCommandGroup(
            m_setHighSpeedCmd,
            new CoralEjectCommand(this),
            new ElevatorGoToCommand(this, Constants.OperatorConstants.ELevels.L1),   // Will use algaeL2_3 if too high
            new WaitCommand(0.4),
            new ElevatorGoToCommand(this, Constants.OperatorConstants.ELevels.L1)
        )
    );
    primary.b().onTrue(new CoralPrepCommand(this));
    primary.x().onTrue(new ClimbDeployCommand(this));
    // primary.y().onTrue(new InstantCommand())
    
    primary.back().onTrue(new ClimbRetractCommand(this));
    primary.start().onTrue(m_toggleSlowSpeed);
    primary.leftBumper().onTrue(m_toggleFieldRelative);

//TODO ADD THE FOLLOW PATH COMMAND

    primary.povUp().onTrue(new StopAllCommand(this));
    primary.povLeft().onTrue(m_resetOdo);
}

public void ConfigButtonBoxBindings(){
    var buttonBox = m_buttonController;
      // Physical layout and Xbox assignment
  // ┌───────┌───────┬───────┐───────┐
  // │Green1 │White2 │ Blue2 │Green1 │
  // │  X    │  Back │ Start │  DU   │
  // ├───────├───────┼───────┤───────┤
  // │Yellow1│Green2 │ Red2  │ Blue3 │
  // │  Y    │  LS   │  RS   │  DD   │
  // ├───────├───────┼───────┤───────┤
  // │ Blue1 │Black2 │Yellow2│ Red3  │
  // │  RB   │  B    │  A    │  DR   │
  // ├───────┼───────┼───────┤───────┤
  // │Black1 │White1 │ Red1  │Yellow3│        
  // │  LB   │   LT  │  RT   │  DL   │        
  // └───────┴───────┴───────┘───────┘  

  buttonBox.x().onTrue(m_setL4);
  buttonBox.y().onTrue(new SequentialCommandGroup(
      m_elevL3
    , m_setL3
  ));
  buttonBox.rightBumper().onTrue(new SequentialCommandGroup(
      m_elevL2
    , m_setL2
  ));
  buttonBox.leftBumper().onTrue(new SequentialCommandGroup(
      new ElevatorGoToCommand(this, Constants.OperatorConstants.ELevels.L1)  // ElevatorGoToCommand will use algaeL2_3 if too high
    , new WaitCommand(0.4)
    , new ElevatorGoToCommand(this, Constants.OperatorConstants.ELevels.L1)
  ));

  buttonBox.back().onTrue(m_elevRelPosUp);
  buttonBox.leftStick().onTrue(m_elevRelPosDown);
  buttonBox.b().onTrue(new SequentialCommandGroup(
      m_elevL1
    , new CoralIntakeCommand(this)
    , m_elevL3
  ));
  buttonBox.leftTrigger().onTrue(m_setLeft);

  buttonBox.start().onTrue(new SequentialCommandGroup(
      m_elevL3_4
    , m_coralDeployManipAlgae
    , m_coralAlgaeRemove
  ));
  buttonBox.start().onFalse(new SequentialCommandGroup(
    m_coralStop
  , m_coralRetractManip
  ));
  buttonBox.rightStick().onTrue(new SequentialCommandGroup(
      m_elevL2_3
    , m_coralDeployManipAlgae
    , m_coralAlgaeRemove
  ));
  buttonBox.rightStick().onFalse(new SequentialCommandGroup(
    m_coralStop
  , m_coralRetractManip
  ));
//   buttonBox.A().OnTrue(frc2::SequentialCommandGroup{      // Score
//       m_FollowPathLED
//     , DeferredCommand(GetFollowPathCommand, {&m_drive} )
//     , CoralPrepCommand(*this)
//     , ConditionalCommand (SequentialCommandGroup{
//                                 ElevatorGoToCommand(*this, L4, c_bUsePresetLevel)
//                               , InstantCommand{[this] {m_coral.DeployManipulator(); }, {&m_coral} } 
//                               , WaitCommand(c_coralDeployDelay)
//                               }
//                           , InstantCommand{[this] {m_coral.RetractManipulator(); }, {&m_coral} }, 
//                           [this](){return m_elevator.GetPresetLevel() == L4;})
// //                          [this](){return (m_elevator.GetPresetLevel() == L4 || m_elevator.GetPresetLevel() == L1);})
//     , CoralEjectCommand(*this)
//     , WaitCommand(c_coralPostEjectDelay)
//     // , CoralEjectPostCommand(*this)
//     , m_elevL1
//     , m_EndLED
//   }.ToPtr());

//TODO FIGURE THIS OUT!!!!!!!!

// () -> m_drive.ToggleSlowSpeed(), m_drive     new InstantCommand(() -> m_coral.DeployManipulatorAlgae(), m_coral);

  buttonBox.rightTrigger().onTrue(m_setRight);

  buttonBox.povUp().onTrue(m_climberDeployRelUp);        // Jog out
  buttonBox.povDown().onTrue(m_climberDeployRelDown);    // Jog in
  buttonBox.povRight().onTrue(
    new SequentialCommandGroup(
        m_driveStop,
        new CoralPrepCommand(this),
        new ConditionalCommand(
            new SequentialCommandGroup(
                new ElevatorGoToCommand(this, Constants.OperatorConstants.ELevels.L4, true),
                new InstantCommand(() -> m_coral.DeployManipulator(), m_coral)
            ),
            new InstantCommand(() -> m_coral.RetractManipulator(), m_coral),
            () -> (m_elevator.GetPresetLevel() == Constants.OperatorConstants.ELevels.L4 || m_elevator.GetPresetLevel() == Constants.OperatorConstants.ELevels.L1)
        ),
        new WaitCommand(c_coralDeployDelay),
        new CoralEjectCommand(this),
        new WaitCommand(c_coralPostEjectDelay),
        new CoralEjectPostCommand(this),
        m_elevL1
    )
);

buttonBox.povLeft().onTrue(m_coralAlgaeRemove);
  
}
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return m_chooser.getSelected();
  }

  public void SetSideSelected(ESideSelected sideSelected){
    m_sideSelected = sideSelected;
  }

  /** Return a Command that follows an on-the-fly PathPlanner path to the currently imaged tag. */
public Command GetFollowPathCommandImpl() {
  // Copy the pattern used in the C++ code: ask for a tag pose and bail out if none found.
  Pose2d targetPose = m_targetPose;
  if (!IsTagBeingImaged()) {
      // Commands.print constructs a command that prints a message and finishes. (WPILib)
      targetPose = GetTagPose();
      return Commands.print("No tag being imaged right now");
  }

  targetPose = GetTagPose();
  // If very close, run the PositionPIDCommand instead (same logic as #define USE_POSITION_PID)
  double pathLen = m_drive.GetCurrentPose().getTranslation().getDistance(targetPose.getTranslation());
  if (pathLen < 0.01) { // 0.01 m == 1 cm threshold (C++ used 0.01; same here)
      System.out.printf("pathLen %.3f running PositionPIDCommand\n", pathLen);
      return new PositionPIDCommand(this, targetPose); // assumes you have a Java constructor
  }

  // Read tuning from SmartDashboard just like your C++ code
  double p = SmartDashboard.getNumber("PathTransP", 5.0);

  // Build a PPHolonomicDriveController (translation PID, rotation PID)
  PPHolonomicDriveController holonomicController =
      new PPHolonomicDriveController(
          new PIDConstants(p, 0.0, 0.0),
          new PIDConstants(5.0, 0.0, 0.0)
      );

  // FollowPathCommand constructor (PathPlanner Java) takes:
  // (PathPlannerPath, Supplier<Pose2d>, Supplier<ChassisSpeeds>, BiConsumer<ChassisSpeeds,DriveFeedforwards>,
  //  PathFollowingController, RobotConfig, BooleanSupplier mirrorForRed, Subsystem... requirements)
  return new FollowPathCommand(
      GetOnTheFlyPath(),                   // PathPlannerPath (on-the-fly)
      () -> m_drive.GetPose(),             // supplier for current robot pose
      () -> m_drive.GetChassisSpeeds(),    // supplier for current chassis speeds
      (ChassisSpeeds speeds, DriveFeedforwards dffs) -> m_drive.Drive(speeds, dffs), // output consumer
      holonomicController,                 // PathFollowingController (PPHolonomicDriveController implements it)
      m_drive.GetRobotCfg(),               // RobotConfig from your drive
      () -> {                              // mirror-for-red boolean supplier (DriverStation.getAlliance() -> Optional)
          Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
      },
      m_drive                              // drive subsystem requirement
  );
}

/** Build a PathPlannerPath on-the-fly between the current robot pose and the currently imaged tag pose. */
public PathPlannerPath GetOnTheFlyPath() {
  PathPlannerPath path = null;

  // Get target pose from your GetTagPose(...) helper (C++ used a reference param)
  Pose2d targetPose = m_targetPose;
  if (!IsTagBeingImaged()) {
      targetPose = GetTagPose();
      return path; // null -> caller should handle this (same as C++ returning nullptr)
  }

  targetPose = GetTagPose();

  // Extract useful values (Java Pose2d stores meters / Rotation2d)
  Pose2d currentPose = m_drive.GetCurrentPose();

  // use chassis speeds to compute heading from velocity vector.
  // In Java ChassisSpeeds exposes vx/vy as vxMetersPerSecond / vyMetersPerSecond.
  ChassisSpeeds cs = m_drive.GetChassisSpeeds();
  double xSpeed = cs.vxMetersPerSecond;
  double ySpeed = cs.vyMetersPerSecond;

  // Compute heading from velocity using atan2(y, x) -> radians, then wrap in a Rotation2d
  double startAngleRad = Math.atan2(ySpeed, xSpeed);
  Rotation2d startRot = new Rotation2d(startAngleRad);

  // Construct the list of poses used for waypointsFromPoses
  List<Pose2d> poses = List.of(
      new Pose2d(currentPose.getX(), currentPose.getY(), startRot),
      targetPose
  );

  // Put some debugging numbers on the dashboard (like the C++ version did)
  SmartDashboard.putNumber("OTFPInitSpeed", m_drive.GetSpeed());
  SmartDashboard.putNumber("OTFPInitRot", currentPose.getRotation().getDegrees());

  // Build the PathPlannerPath using the simplified constructor:
  // PathPlannerPath(List<Waypoint>, PathConstraints, IdealStartingState, GoalEndState)
  path = new PathPlannerPath(
      PathPlannerPath.waypointsFromPoses(poses),
      m_pathConstraints,
      new IdealStartingState(m_drive.GetSpeed(), currentPose.getRotation()),
      new GoalEndState(0.0, targetPose.getRotation()) // end with 0 m/s and target rotation
  );

  // Prevent the path from being auto-flipped by the library (same as C++: path->preventFlipping = true)
  path.preventFlipping = true;

  return path;
}

public void ConfigureRobotLEDs(){
  GetLED().periodic();
}

//_____________________________________________________________________________________________________________________





  /** The container for the robot. Contains subsystems, OI devices, and commands. */




}
