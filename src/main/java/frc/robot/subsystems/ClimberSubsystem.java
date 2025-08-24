package frc.robot.subsystems; // change to your package

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

/**
 * Java translation of your C++ ClimberSubsystem
 * Keeps original names: Periodic(), Set(...), GoToPosition(...), GoToPositionRel(...), GetPosition()
 */
public class ClimberSubsystem extends SubsystemBase {
  // constants (kept the same names as in your header)
  public static final double c_defaultClimbResetTurns = -20.0;
  public static final double c_defaultClimbDeployTurns = 370.0;
  public static final double c_defaultClimbDeployRelTurns = 10.0;
  public static final double c_defaultClimbSpringTensionTurns = 100.0;

  // PID slots (mirror your C++ constants)
  private static final ClosedLoopSlot c_defaultClimbDownPIDSlot = ClosedLoopSlot.kSlot0;
  private static final ClosedLoopSlot c_defaultClimbUpPIDSlot = ClosedLoopSlot.kSlot1;

  private static final double c_defaultClimbDownP = 0.006;
  private static final double c_defaultClimbUpP = 0.1;
  private static final double c_defaultClimbI = 0.0;
  private static final double c_defaultClimbD = 0.0;
  private static final double c_defaultClimbFF = 0.00000;

  // member variables (kept the same names)
  private final SparkMax m_motor;
  private final RelativeEncoder m_relativeEnc;
  private final SparkClosedLoopController m_closedLoopController;

  private double m_climbPosition = 1.0;
  private final DoubleLogEntry m_log;

  private double m_direction = 1.0;

  // small state used by Periodic for change-detection (mirrors static locals in C++)
  private int m_periodicCounter = 0;
  private double m_lastDownP = 0.0;
  private double m_lastUpP = 0.0;
  private double m_lastI = 0.0;
  private double m_lastD = 0.0;
  private double m_lastFF = 0.0;

  /**
   * Constructor. This configures the SparkMax device using SparkMaxConfig (the
   * SparkBaseConfig-derived concrete config class). This avoids trying to
   * instantiate the abstract SparkBaseConfig.
   */
  public ClimberSubsystem() {
    // replace this CAN ID with your constant: kClimbMotorCANID
    m_motor = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);

    // Use the SPARK MAX config class (do NOT try to 'new SparkBaseConfig()' — it's the abstract base).
    SparkMaxConfig config = new SparkMaxConfig();

    // Set the idle mode & inversion on the config (chain calls are supported)
    // Note: method names are the Java/REVLib ones: idleMode(...) and inverted(...)
    config.idleMode(SparkBaseConfig.IdleMode.kBrake)
          .inverted(false);

    // closed-loop & ramp configuration
    config.closedLoopRampRate(0.0);
    // set output range for closed-loop default slot (ClosedLoopConfig.outputRange exists)
    config.closedLoop.outputRange(-1.0, 1.0);

    // Apply the configuration and persist it (the same flags you used in C++)
    m_motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // Get encoder & closed-loop controller references
    m_relativeEnc = m_motor.getEncoder();
    m_closedLoopController = m_motor.getClosedLoopController();

    // reset encoder position as you did in C++
    m_relativeEnc.setPosition(0.0);

    // Data logging entry (DataLogManager & DoubleLogEntry)
    DataLog log = DataLogManager.getLog();
    m_log = new DoubleLogEntry(log, "/climber/position");

    // Dashboard / Preferences initialization (mirror the C++ constructor)
    SmartDashboard.putNumber("ClimbPosEcho", 1.0);
    SmartDashboard.putNumber("ClimbRel", c_defaultClimbDeployRelTurns);

    Preferences.initDouble("kClimbPosDownP", c_defaultClimbDownP);
    Preferences.initDouble("kClimbPosUpP", c_defaultClimbUpP);
    Preferences.initDouble("kClimbPosI", c_defaultClimbI);
    Preferences.initDouble("kClimbPosD", c_defaultClimbD);
    Preferences.initDouble("kClimbPosFF", c_defaultClimbFF);
  }

  /**
   * Periodic — runs regularly on the scheduler
   * Mirrors the C++ logic: every 20 loops, reload the PID prefs and reconfigure the motor if changed.
   */
  @Override
  public void periodic() {
    if ((m_periodicCounter++ % 20) == 0) {
      double pDown = Preferences.getDouble("kClimbPosDownP", c_defaultClimbDownP);
      double pUp   = Preferences.getDouble("kClimbPosUpP",   c_defaultClimbUpP);
      double i     = Preferences.getDouble("kClimbPosI",      c_defaultClimbI);
      double d     = Preferences.getDouble("kClimbPosD",      c_defaultClimbD);
      double ff    = Preferences.getDouble("kClimbPosFF",     c_defaultClimbFF);

      // If P changed for down slot, apply via config.closedLoop.p(...)
      if (pDown != m_lastDownP) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.p(pDown, c_defaultClimbDownPIDSlot);
        m_motor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      }

      // If P changed for up slot, apply
      if (pUp != m_lastUpP) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.p(pUp, c_defaultClimbUpPIDSlot);
        m_motor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      }

      // I for both slots (as in C++)
      if (i != m_lastI) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.i(i, c_defaultClimbDownPIDSlot).i(i, c_defaultClimbUpPIDSlot);
        m_motor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      }

      // D for both slots
      if (d != m_lastD) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.closedLoop.d(d, c_defaultClimbDownPIDSlot).d(d, c_defaultClimbUpPIDSlot);
        m_motor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      }

      // FF for both slots (note: REV API may deprecate velocityFF in favor of feedForward; many examples still use velocityFF)
      if (ff != m_lastFF) {
        SparkMaxConfig cfg = new SparkMaxConfig();
        // closedLoop.velocityFF(...) exists in javadocs (deprecated name), or use feedForward config if available
        cfg.closedLoop.velocityFF(ff, c_defaultClimbDownPIDSlot).velocityFF(ff, c_defaultClimbUpPIDSlot);
        m_motor.configure(cfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      }

      // save last values
      m_lastDownP = pDown;
      m_lastUpP   = pUp;
      m_lastI     = i;
      m_lastD     = d;
      m_lastFF    = ff;

      // dashboard & logging echoes (mirrors C++)
      SmartDashboard.putNumber("ClimbPosEcho", m_relativeEnc.getPosition());
      m_log.append(m_relativeEnc.getPosition());
    }
  }

  /** Drives the climber at a given speed. */
  public void Set(double speed) {
    m_motor.set(speed); // SparkMax.set(double) is the open-loop output set
  }

  /** Stops the motor (keeps your name Stop() same as C++). */
  public void Stop() {
    m_motor.stopMotor();
  }

  /** Go to absolute position (rotations/turns depending on encoder scale in your config). */
  public void GoToPosition(double position) {
    // Use setSetpoint(...) on SparkClosedLoopController (preferred modern API)
    m_closedLoopController.setReference(position, SparkBase.ControlType.kPosition, c_defaultClimbUpPIDSlot);
    // mirror C++: it set the up slot explicitly — adapt if you need different slot selection logic
  }

  /** Go to relative position (current + delta). */
  public void GoToPositionRel(double rel) {
    GoToPosition(m_relativeEnc.getPosition() + rel);
  }

  /** Return current encoder position. */
  public double GetPosition() {
    return m_relativeEnc.getPosition();
  }
}
