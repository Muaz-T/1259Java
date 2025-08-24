// DriveSubsystem.java
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Your interfaces and custom classes
import frc.robot.IDriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.PigeonGyro;

// LimelightHelpers: drop the official LimelightHelpers.java into your project (as instructed by Limelight)
// and import it. It typically lives in your robot package (class name: LimelightHelpers).
import frc.robot.LimelightHelpers;

/**
 * Represents a swerve drive style DriveSubsystem (Java port from C++).
 */
public class DriveSubsystem extends SubsystemBase implements IDriveSubsystem {
  // ===== Static constants (units converted to plain SI doubles for Java) =====
  // Thrifty 18P13 Kraken true max speed (16.8 ft/s)
  private static final double kMaxSpeed = Units.feetToMeters(16.8);    // m/s
  private static final double kSlowSpeed = Units.feetToMeters(1.0);    // m/s

  private static final double kMaxAngularSpeed = 2.5 * Math.PI;        // rad/s
  private static final double kLowAngularSpeed  = Math.PI / 3.0;       // rad/s

  private static final double kRotationDriveMaxSpeed = 7.5;            // rad/s
  private static final double kRotationDriveDirectionLimit = 7.0;      // rad/s

  private static final double kAimingRotationDriveMaxSpeed = 7.5;      // rad/s
  private static final double kAimingRotationDriveDirectionLimit = 7.0;// rad/s

  // Robot geometry (24 in x 24 in)
  private static final double kTrackWidth = Units.inchesToMeters(24.0); // meters
  private static final double kWheelBase  = Units.inchesToMeters(24.0); // meters

  // Robot mass & moment of inertia (ported calc)
  private static final double c_RobotMassKg = Units.lbsToKilograms(140.0);
  // Using the same MOI structure as C++: (m * (a^2 + a^2))/12  with a^2 term ~0.7903212 m^2 from original
  private static final double c_MOI = (c_RobotMassKg * (0.7903212 + 0.7903212)) / 12.0; // kg·m^2

  // ===== Kinematic locations (match C++ order) =====
  private final Translation2d m_frontLeftLocation  = new Translation2d(  kWheelBase / 2.0,  kTrackWidth / 2.0);
  private final Translation2d m_frontRightLocation = new Translation2d(  kWheelBase / 2.0, -kTrackWidth / 2.0);
  private final Translation2d m_rearLeftLocation   = new Translation2d( -kWheelBase / 2.0,  kTrackWidth / 2.0);
  private final Translation2d m_rearRightLocation  = new Translation2d( -kWheelBase / 2.0, -kTrackWidth / 2.0);

  // Offsets from Preferences (keep keys identical)
  private final double kFLoffset = Preferences.getDouble("Offset1", 0.0);
  private final double kFRoffset = Preferences.getDouble("Offset2", 0.0);
  private final double kBLoffset = Preferences.getDouble("Offset4", 0.0);
  private final double kBRoffset = Preferences.getDouble("Offset3", 0.0);

  // ===== Hardware / modules =====
  private final SwerveModule m_frontLeft  = new SwerveModule(1,  2,  kFLoffset, true);   // 1
  private final SwerveModule m_frontRight = new SwerveModule(3, 4, kFRoffset, false); // 2
  private final SwerveModule m_rearLeft   = new SwerveModule(5,   6,   kBLoffset, true);  // 4
  private final SwerveModule m_rearRight  = new SwerveModule(7,  8,  kBRoffset, false); // 3

  private final PigeonGyro m_gyro = new PigeonGyro(1);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation, m_rearRightLocation);

  // PathPlanner on-the-fly config (use existing SwerveModule fields/constants)
  private final ModuleConfig m_moduleCfg = new ModuleConfig(
      0.0508,          // wheel radius (m)
      kMaxSpeed * 0.85,                   // true max module speed (m/s)
      1.0,                                // wheel COF
      DCMotor.getKrakenX60(1),            // motor (num motors per module)
      6.28,       // drive reduction
      95.0,                               // drive current limit (A)
      1                                   // num motors
  );

  private final RobotConfig m_robotConfig = new RobotConfig(
      c_RobotMassKg,
      c_MOI,
      m_moduleCfg,
      new Translation2d[] { m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation, m_rearRightLocation }
  );

  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.GetRotation2d(), // do NOT reset gyro; estimator tracks offset
      new SwerveModulePosition[] {
        m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_rearLeft.GetPosition(), m_rearRight.GetPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.1, 0.1, 0.1),          // state std devs (x, y, theta)
      VecBuilder.fill(0.7, 0.7, 9_999_999.0)   // vision std devs (x, y, theta)
  );

  private final PIDController m_rotationPIDController = new PIDController(1.0, 0.0, 0.025);

  private boolean m_bOverrideXboxInput = false;
  private int m_AdjustingWheelAngleCount = 0;

  // Logging
  private final DoubleLogEntry m_logRobotPoseX;
  private final DoubleLogEntry m_logRobotPoseY;
  private final DoubleLogEntry m_logRobotPoseTheta;
  private final DoubleLogEntry m_logGyroPitch;
  private final DoubleLogEntry m_logDriveInputX;
  private final DoubleLogEntry m_logDriveInputY;
  private final DoubleLogEntry m_logDriveInputRot;

  // Public to match your C++ visibility (values in SI)
  public double m_currentMaxSpeed = kSlowSpeed; //kMaxSpeed;          // start slow like C++
  public double m_currentMaxAngularSpeed = kMaxAngularSpeed;

  public DriveSubsystem() {
    // DataLog setup
    DataLog log = DataLogManager.getLog();
    m_logRobotPoseX    = new DoubleLogEntry(log, "/odometry/robotPoseX");
    m_logRobotPoseY    = new DoubleLogEntry(log, "/odometry/robotPoseY");
    m_logRobotPoseTheta= new DoubleLogEntry(log, "/odometry/robotPoseTheta");
    m_logGyroPitch     = new DoubleLogEntry(log, "/drivegyro/pitch");
    m_logDriveInputX   = new DoubleLogEntry(log, "/input/X");
    m_logDriveInputY   = new DoubleLogEntry(log, "/input/Y");
    m_logDriveInputRot = new DoubleLogEntry(log, "/input/Rot");

    // Build info & preference defaults (mirror C++)
    Preferences.setString("BuildDate", java.time.LocalDate.now().toString());
    Preferences.setString("BuildTime", java.time.LocalTime.now().toString());
    Preferences.initString("Name", "ThingX");
    Preferences.initDouble("Offset1", 0.0);
    Preferences.initDouble("Offset2", 0.0);
    Preferences.initDouble("Offset3", 0.0);
    Preferences.initDouble("Offset4", 0.0);
  }

  // ---- Helper: returns states in C++ order FL, FR, BL, BR ----
  private SwerveModuleState[] GetModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()
    };
  }

  // ========================= Public API (names preserved) =========================

  // C++: void Drive(const frc::ChassisSpeeds& speeds, const DriveFeedforwards& dffs)
  public void Drive(ChassisSpeeds speeds, DriveFeedforwards dffs) {
    // Mirror the axis flip from C++
    Drive(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
  }

  // C++: Drive(units::mps xSpeed, units::mps ySpeed, units::radps rot, bool fieldRelative)
  public void Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    m_logDriveInputX.append(xSpeed);
    m_logDriveInputY.append(ySpeed);
    m_logDriveInputRot.append(rot);

    // Keep max speed synced to current mode
    // m_frontLeft.SetMaxSpeed(m_currentMaxSpeed);
    // m_frontRight.SetMaxSpeed(m_currentMaxSpeed);
    // m_rearLeft.SetMaxSpeed(m_currentMaxSpeed);
    // m_rearRight.SetMaxSpeed(m_currentMaxSpeed);

    if (!m_bOverrideXboxInput && m_AdjustingWheelAngleCount == 0) {
      ChassisSpeeds chassisSpeeds;
      var allianceOpt = DriverStation.getAlliance();
      if (fieldRelative) {
        boolean blue = allianceOpt.isPresent() && allianceOpt.get() == DriverStation.Alliance.Blue;
        if (blue) {
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, rot, m_poseEstimator.getEstimatedPosition().getRotation());
        } else {
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              -xSpeed, -ySpeed, rot, m_poseEstimator.getEstimatedPosition().getRotation());
        }
      } else {
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
      }

      SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, m_currentMaxSpeed);

      // Order preserved: FL, FR, BL, BR
      m_frontLeft.SetDesiredState(states[0]);
      m_frontRight.SetDesiredState(states[1]);
      m_rearLeft.SetDesiredState(states[2]);
      m_rearRight.SetDesiredState(states[3]);
    }

    if (m_AdjustingWheelAngleCount > 0) m_AdjustingWheelAngleCount--;
  }

  // Field-relative RotationDrive with explicit target heading (C++: units::radian_t rot)
  public void RotationDrive(double xSpeed, double ySpeed, Rotation2d rot, boolean fieldRelative) {
    double error = rot.getRadians() - m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
    SmartDashboard.putNumber("turnError", error);
    if (error > Math.PI)  error -= 2.0 * Math.PI;
    else if (error < -Math.PI) error += 2.0 * Math.PI;

    double max = kRotationDriveMaxSpeed;
    double maxTurn = kRotationDriveDirectionLimit;

    boolean isAiming = SmartDashboard.getBoolean("IsAiming", false);
    if (isAiming) {
      max = kAimingRotationDriveMaxSpeed;
      // maxTurn = kAimingRotationDriveDirectionLimit; // left as in C++
    }

    // PID on heading error: Calculate(measurement=0, setpoint=error)
    double desiredTurnRate = m_rotationPIDController.calculate(0.0, error); // rad/s

    // Convert gyro rate to rad/s for consistent comparison
    double currentTurnRateRad = Units.degreesToRadians(m_gyro.GetTurnRate()); // C++ code compared to 720 deg/s

    // Prevent sharp turning if already fast in opposite direction
    if (Math.abs(currentTurnRateRad) >= maxTurn
        && Math.signum(desiredTurnRate) != Math.signum(currentTurnRateRad)) {
      desiredTurnRate *= -1.0;
    }

    // Power limiting (clamp to ±max)
    if (Math.abs(desiredTurnRate) > max) {
      desiredTurnRate = Math.signum(desiredTurnRate) * max;
    }

    Drive(xSpeed, ySpeed, desiredTurnRate, fieldRelative);
  }

  // Field-relative RotationDrive from joystick vector (C++ overload with xRot, yRot)
  public void RotationDrive(double xSpeed, double ySpeed, double xRot, double yRot, boolean fieldRelative) {
    if (xRot != 0.0 || yRot != 0.0) {
      Rotation2d rot = new Rotation2d(Math.atan2(yRot, xRot));
      RotationDrive(xSpeed, ySpeed, rot, fieldRelative);
    } else {
      Drive(xSpeed, ySpeed, 0.0, fieldRelative);
    }
  }

  // C++: void Drive(const frc::ChassisSpeeds& speeds, const DriveFeedforwards& dffs) already above

  @Override
  public void periodic() {
    UpdateOdometry();

    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();

    // Log odometry values
    Pose2d pose = m_poseEstimator.getEstimatedPosition();
    m_logRobotPoseX.append(pose.getX());
    m_logRobotPoseY.append(pose.getY());
    m_logRobotPoseTheta.append(pose.getRotation().getDegrees());
    m_logGyroPitch.append(m_gyro.GetPitch());

    SmartDashboard.putNumber("azimuthDeg", m_gyro.GetRotation2d().getDegrees());
    SmartDashboard.putNumber("GyroYaw", m_gyro.GetYaw());
    SmartDashboard.putBoolean("SlowSpeed", m_currentMaxSpeed == kSlowSpeed);

    // Limelight MegaTag2 vision update (guarded similar to your C++)
      // Use pose estimator rotation since we are not resetting the gyro
      LimelightHelpers.SetRobotOrientation("limelight-reef",
          pose.getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

      boolean doUpdate = true;
      LimelightHelpers.PoseEstimate mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-reef");

      if (mt2.tagCount == 0 || Math.abs(m_gyro.GetTurnRate()) > 720.0) { // 720 deg/s threshold as in C++
        doUpdate = false;
      }

      if (doUpdate) {
        m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      }
  }

  // ===== Methods matching your C++ names/signatures (types adapted to Java) =====
  @Override
  public void UpdateOdometry() {
    Pose2d pose = m_poseEstimator.update(
        m_gyro.GetRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
          m_rearLeft.GetPosition(),  m_rearRight.GetPosition()
        });

    SmartDashboard.putNumber("PoseX", pose.getX());
    SmartDashboard.putNumber("PoseY", pose.getY());
    SmartDashboard.putNumber("PoseRot", pose.getRotation().getDegrees());
  }

  @Override
  public void ResetOdometry(Pose2d pose) {
    System.out.printf("resetx %.3f resety %.3f resetrot %.3f%n",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    // Do not set the gyro; pose estimator keeps track of the offset
    // Java API requires full reset with current module positions:
    m_poseEstimator.resetPose(pose);
  }

  @Override
  public double GetPitch() { // returning degrees would lose type; use Rotation2d for Java, but keep name
    // C++ returned units::degree_t; in Java, expose as Rotation2d from degrees
    return m_gyro.GetPitch();
  }

  @Override
  public Pose2d GetPose() { return m_poseEstimator.getEstimatedPosition(); }

  @Override
  public ChassisSpeeds GetChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(GetModuleStates());
  }

  public double GetSpeed() {
    ChassisSpeeds robotRel = m_kinematics.toChassisSpeeds(GetModuleStates());
    ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRel, m_poseEstimator.getEstimatedPosition().getRotation());
    double vx = fieldVel.vxMetersPerSecond;
    double vy = fieldVel.vyMetersPerSecond;
    return Math.hypot(vx, vy);
  }

  @Override
  public void SetModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_currentMaxSpeed);
    m_frontLeft.SetDesiredState(desiredStates[0]);
    m_frontRight.SetDesiredState(desiredStates[1]);
    m_rearRight.SetDesiredState(desiredStates[3]);
    m_rearLeft.SetDesiredState(desiredStates[2]);
  }

  public Pose2d GetCurrentPose() { return m_poseEstimator.getEstimatedPosition(); }
  @Override public double GetX() { return m_poseEstimator.getEstimatedPosition().getX(); }
  @Override public double GetY() { return m_poseEstimator.getEstimatedPosition().getY(); }

  @Override
  public void ResyncAbsRelEnc() {
    m_frontLeft.resyncAbsRelEnc();
    m_frontRight.resyncAbsRelEnc();
    m_rearLeft.resyncAbsRelEnc();
    m_rearRight.resyncAbsRelEnc();
  }

  @Override public void SetOverrideXboxInput(boolean bOverride) { m_bOverrideXboxInput = bOverride; }

  public void JogRotate(boolean bClockwise) {
    Rotation2d angle = Rotation2d.fromDegrees(5.0);
    Rotation2d rot = bClockwise ? angle : angle.unaryMinus();
    double rotRate = rot.getRadians() / 1.0; // per second
    Drive(0.0, 0.0, rotRate, false);
  }

  public void DriveBack() {
    m_bOverrideXboxInput = true;
    SwerveModuleState sms = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    SetAllDesiredState(sms);
  }

  @Override public void WheelsForward()  { setAllWheelsToAngleDegrees(  0.0); }
  @Override public void WheelsLeft()     { setAllWheelsToAngleDegrees( 90.0); }
  @Override public void WheelsBackward() { setAllWheelsToAngleDegrees(180.0); }
  @Override public void WheelsRight()    { setAllWheelsToAngleDegrees(-90.0); }

  public void Stop() { m_bOverrideXboxInput = false; }

  public TalonFX GetTalon(int module) {
    switch (module) {
      case 0: return m_frontLeft.getTalon();
      case 1: return m_frontRight.getTalon();
      case 2: return m_rearRight.getTalon();
      default: return m_rearLeft.getTalon();
    }
  }

  public Rotation2d GetGyroAzimuthDeg() { return m_gyro.GetRotation2d(); }

  public RobotConfig GetRobotCfg() { return m_robotConfig; }

  public void SetSlowSpeed(boolean slow) {
    m_currentMaxSpeed = slow ? kSlowSpeed : kMaxSpeed;
    m_currentMaxAngularSpeed = slow ? kLowAngularSpeed : kMaxAngularSpeed;

    m_frontLeft.SetMaxSpeed(m_currentMaxSpeed);
    m_frontRight.SetMaxSpeed(m_currentMaxSpeed);
    m_rearLeft.SetMaxSpeed(m_currentMaxSpeed);
    m_rearRight.SetMaxSpeed(m_currentMaxSpeed);
  }

  @Override
  public void ToggleSlowSpeed() {
    SetSlowSpeed(m_currentMaxSpeed == kMaxSpeed);
  }

  // ========================= Private helpers =========================
  private void SetAllDesiredState(SwerveModuleState sms) {
    m_frontLeft.SetDesiredState(sms);
    m_frontRight.SetDesiredState(sms);
    m_rearLeft.SetDesiredState(sms);
    m_rearRight.SetDesiredState(sms);
  }

  private void setAllWheelsToAngleDegrees(double deg) {
    m_AdjustingWheelAngleCount = 5;
    SwerveModuleState sms = new SwerveModuleState(0.0, Rotation2d.fromDegrees(deg));
    SetAllDesiredState(sms);
  }

}
