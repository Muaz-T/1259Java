// package: change this to your robot package
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;

// REV / Spark imports (2024/2025 REVLib)
// adjust imports if your REV package path differs (some projects use com.revrobotics.*)
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig; // for IdleMode enum

// Your constants classes - adjust package if needed
import frc.robot.Constants;

/**
 * Java translation of CoralManipulatorSubsystem (C++ -> Java).
 * Matches original names and behavior while using the Java REV/WPILib APIs.
 */
public class CoralManipulatorSubsystem extends SubsystemBase {
  // Constants kept from your C++ header
  public static final double c_servoDeployDefault = 0.8;
  public static final double c_servoRetractDefault = 0.3;

  // PID slot constants (mirror original)
  private static final ClosedLoopSlot c_intakeGeneralPIDSlot = ClosedLoopSlot.kSlot0;
  private static final ClosedLoopSlot c_intakeExtendPIDSlot  = ClosedLoopSlot.kSlot1;

  private static final double c_defaultCoralManipP = 0.1;
  private static final double c_defaultCoralManipI = 0.0;
  private static final double c_defaultCoralManipD = 0.0;

  // Hardware + sensors (kept same names)
  private final Timer m_timer = new Timer();
  private final DigitalInput m_photoEyeIn;
  private final DigitalInput m_photoEyeOut;
  private final Servo m_deployServo;

  private final SparkMax m_coralMotor;
  private final RelativeEncoder m_coralRelativeEnc;
  private final SparkClosedLoopController m_coralPIDController;

  private final DoubleLogEntry m_log;

  // fields to mirror C++ behaviour
  private double lastP = 0.0;
  private double lastI = 0.0;
  private double lastD = 0.0;

  /**
   * Constructor.
   * - configures SparkMax via SparkMaxConfig (REV recommended API).
   * - gets encoder + closed-loop controller references.
   */
  public CoralManipulatorSubsystem() {
    // instantiate IO using your constants (change class names if needed)
    m_photoEyeIn  = new DigitalInput(0);
    m_photoEyeOut = new DigitalInput(1);
    m_deployServo = new Servo(0);

    // create motor (use your CAN ID constant)
    m_coralMotor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);

    // Configure the Spark using the SparkMaxConfig object (the Java equivalent of the C++ SparkBaseConfig usage)
    SparkMaxConfig config = new SparkMaxConfig();
    // Idle mode (Brake)
    config.idleMode(SparkBaseConfig.IdleMode.kBrake); // documented in REV config docs. :contentReference[oaicite:0]{index=0}
    // closed-loop ramp / output-range (same idea as the C++; exact accessor is part of the SparkMaxConfig closed-loop config)
    config.closedLoop.outputRange(-1.0, 1.0); // set output range for closed-loop slot(s). :contentReference[oaicite:1]{index=1}
    config.closedLoopRampRate(0.0); // no-op call shown in docs (keeps API accessible) — you can set motion params if required. :contentReference[oaicite:2]{index=2}

    // Apply configuration (ResetMode / PersistMode map to the same REV API)
    m_coralMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    // Get encoder & controller AFTER motor is created
    m_coralRelativeEnc = m_coralMotor.getEncoder(); // returns rotations by default. Use conversion factors if needed. :contentReference[oaicite:3]{index=3}
    m_coralPIDController = m_coralMotor.getClosedLoopController();

    // reset encoder to 0 (same behavior as C++)
    m_coralRelativeEnc.setPosition(0.0);

    // Preferences (mirrors C++ init)
    Preferences.initDouble("kCoralManipP", c_defaultCoralManipP);
    Preferences.initDouble("kCoralManipI", c_defaultCoralManipI);
    Preferences.initDouble("kCoralManipD", c_defaultCoralManipD);

    SmartDashboard.putNumber("CoralRetractTurns", 3.25);
    Preferences.initDouble("ServoDeploy", c_servoDeployDefault);
    Preferences.initDouble("ServoRetract", c_servoRetractDefault);

    // Data log entry (WPILib DataLogManager + DoubleLogEntry)
    m_log = new DoubleLogEntry(DataLogManager.getLog(), "/coral/position"); // see WPILib datalog docs. :contentReference[oaicite:4]{index=4}
  }

  /**
   * Keep original C++ function name Periodic() for familiarity (we'll also override periodic() to call this).
   * This implements the same logic as your C++ Periodic().
   */
  @Override
  public void periodic() {
    LoadDeployPid();

    SmartDashboard.putNumber("ServoDeployEcho", m_deployServo.get()); // Servo.get() returns commanded value. :contentReference[oaicite:5]{index=5}
    SmartDashboard.putNumber("ServoRetractEcho", m_deployServo.get());
    SmartDashboard.putNumber("Coral echo", m_coralRelativeEnc.getPosition()); // rotations by default. :contentReference[oaicite:6]{index=6}
    SmartDashboard.putBoolean("CoralManipPhotoEyeIn", m_photoEyeIn.get());
    SmartDashboard.putBoolean("CoralManipPhotoEyeOut", m_photoEyeOut.get());

    // log encoder position
    m_log.append(m_coralRelativeEnc.getPosition());
  }

  /**
   * WPILib lifecycle method - call the C++-style Periodic() so your original name & logic remain.
   */


  /**
   * Re-implements LoadDeployPid() from C++:
   * - reads Preferences,
   * - if changed, applies a small SparkMaxConfig with the updated closed-loop gain(s).
   *
   * Note: the REV config API's ClosedLoop config lets you set p/i/d per closed-loop slot.
   * Example: config.closedloop.p(p, slot).
   * See REV docs for the Java ClosedLoop configuration accessors. :contentReference[oaicite:7]{index=7}
   */
  private void LoadDeployPid() {
    double p = Preferences.getDouble("kCoralManipP", c_defaultCoralManipP);
    double i = Preferences.getDouble("kCoralManipI", c_defaultCoralManipI);
    double d = Preferences.getDouble("kCoralManipD", c_defaultCoralManipD);

    if (p != lastP) {
      SparkMaxConfig cfg = new SparkMaxConfig();
      cfg.closedLoop.p(p, c_intakeGeneralPIDSlot); // set P for general slot. :contentReference[oaicite:8]{index=8}
      m_coralMotor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
    if (i != lastI) {
      SparkMaxConfig cfg = new SparkMaxConfig();
      // apply I to both extend and general slots like your C++ code
      cfg.closedLoop.i(i, c_intakeExtendPIDSlot).i(i, c_intakeGeneralPIDSlot); 
      m_coralMotor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
    if (d != lastD) {
      SparkMaxConfig cfg = new SparkMaxConfig();
      cfg.closedLoop.d(d, c_intakeExtendPIDSlot).d(d, c_intakeGeneralPIDSlot);
      m_coralMotor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    lastP = p;
    lastI = i;
    lastD = d;
  }

  /**
   * SetFeeder: original C++ had a clamp and commented out motor Set.
   * There's no feeder motor member in the C++ header (it was commented), so keep it a no-op but still clamp.
   */
  public void SetFeeder(double speed) {
    speed = Math.max(-1.0, Math.min(1.0, speed));
    // TODO: if you have a feeder motor (e.g. TalonSRX), call its set method here.
  }

  /** Set the manipulator (open-loop) — uses the SPARK's set() method. */
  public void SetManipulator(double speed) {
    speed = Math.max(-1.0, Math.min(1.0, speed));
    m_coralMotor.set(speed); // open-loop percent [-1..1]
  }

  /** Retract coral by a number of turns depending on ELevels (ELevels is declared in your Constants file) */
  public void RetractCoral(frc.robot.Constants.OperatorConstants.ELevels eLevel) {
    double turns = SmartDashboard.getNumber("CoralRetractTurns", 3.25);
    // If caller requests L4, use the larger retraction (matches C++ logic)
    if (eLevel == frc.robot.Constants.OperatorConstants.ELevels.L4) {
      turns = 8.0;
    }
    // Use deprecated setReference to match your C++ SetReference behavior (position control).
    // The deprecated setReference(double, SparkBase.ControlType) exists in the Java SparkClosedLoopController.
    // See REV docs for `setReference` deprecation note — it still exists and mirrors the C++ behavior.
    m_coralPIDController.setReference(
        m_coralRelativeEnc.getPosition() + turns,
        SparkBase.ControlType.kPosition
    ); // deprecated API usage matches the C++ style. :contentReference[oaicite:9]{index=9}
  }

  public void RetractManipulator() {
    double sp = Preferences.getDouble("ServoRetract", c_servoRetractDefault); m_deployServo.set(sp); 
  }

  /** Go to an absolute position (turns) using closed-loop */
  public void GoToPosition(double turns) {
    // Use the closed-loop controller to move to 'turns' (position mode)
    // We'll use the deprecated setReference overload that takes (double, ControlType).
    m_coralPIDController.setReference(turns, SparkBase.ControlType.kPosition); // deprecated but available. :contentReference[oaicite:10]{index=10}
  }

  /** Return encoder position (rotations by default) */
  public double GetPosition() {
    return m_coralRelativeEnc.getPosition(); // returns rotations by default; conversions available via config. :contentReference[oaicite:11]{index=11}
  }

  /** Photo-eye inputs */
  public boolean IsCoralPresentInput()  { return m_photoEyeIn.get(); }
  public boolean IsCoralPresentOutput() { return m_photoEyeOut.get(); }

  /** Eject coral: set voltage directly (use cautiously). SparkBase.setVoltage exists. */
  public void EjectCoral(boolean slow) {
    double volts = slow ? -5.0 : -9.0;
    m_coralMotor.setVoltage(volts); // set direct voltage (same concept as C++ SetVoltage). :contentReference[oaicite:12]{index=12}
  }

  /** Stop motor */
  public void Stop() {
    m_coralMotor.setVoltage(0.0);
  }

  public void DeployManipulator() {
    double sp = Preferences.getDouble("ServoDeploy", c_servoDeployDefault); 
    m_deployServo.set(sp); 
  }

  public void DeployManipulatorAlgae() {
    double sp = Preferences.getDouble("ServoDeployAlgae", 0.45); 
    m_deployServo.set(0.5); // fixed position for algae removal (matches C++ logic)
  }
}
