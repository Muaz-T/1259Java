// VisionSubsystem.java
package frc.robot.subsystems;  // <-- uncomment and set your package

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// In C++ this was an enum used as raw indices; in Java we mirror
// that behavior with integer constants so m_netBufferAlli[eX] etc. still work.
final class BotPoseIndices {
  public static final int eX = 0;
  public static final int eY = 1;
  public static final int eZ = 2;
  public static final int ePitch = 3;
  public static final int eRoll = 4;
  public static final int eYaw = 5;
  public static final int eLatency = 6;

  private BotPoseIndices() {}
}

public class VisionSubsystem extends SubsystemBase {
  // ----- fields translated 1:1 -----
  private boolean m_isValidReef = false;
  private double[] m_netBufferField = new double[] {0.0, 0.0}; // size placeholder; overwritten by LL arrays
  private double[] m_netBufferAlli  = new double[] {0.0, 0.0}; // size placeholder; overwritten by LL arrays
  private final double[] m_zero_vector = new double[] {42.0, 42.0, 42.0, 92.0, 10.0, 22.0};

  // NetworkTables (Java): NetworkTableInstance.getDefault().getTable("...")
  private final NetworkTable m_netTableReef =
      NetworkTableInstance.getDefault().getTable("limelight-reef");

  private boolean m_bIsBlue = false;
  private double m_tyReef = 0.0;
  private double m_txReef = 0.0;
  private int m_tidReef = -1;

  // WPILib Java DataLog entries
  private DoubleLogEntry m_logRobotAlliPoseX;
  private DoubleLogEntry m_logRobotAlliPoseY;
  private DoubleLogEntry m_logRobotAlliPoseTheta;
  private DoubleLogEntry m_logRobotPoseX;
  private DoubleLogEntry m_logRobotPoseY;
  private DoubleLogEntry m_logRobotPoseTheta;
  private DoubleLogEntry m_logLL_Latency;
  private DoubleLogEntry m_logtxReef;
  private DoubleLogEntry m_logtyReef;
  private IntegerLogEntry m_logtidReef;

  // ----- constructor -----
  public VisionSubsystem() {
    // DriverStation.getAlliance() in Java returns Optional<Alliance>
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      m_bIsBlue = (alliance.get() == Alliance.Blue);
    }

    // DataLog Java API
    DataLog log = DataLogManager.getLog();
    m_logRobotAlliPoseX   = new DoubleLogEntry(log, "/vision/robotAlliPoseX");
    m_logRobotAlliPoseY   = new DoubleLogEntry(log, "/vision/robotAlliPoseY");
    m_logRobotAlliPoseTheta = new DoubleLogEntry(log, "/vision/robotAlliPoseTheta");
    m_logLL_Latency       = new DoubleLogEntry(log, "/vision/LL_Latency");
    m_logRobotPoseX       = new DoubleLogEntry(log, "/vision/robotPoseX");
    m_logRobotPoseY       = new DoubleLogEntry(log, "/vision/robotPoseY");
    m_logRobotPoseTheta   = new DoubleLogEntry(log, "/vision/robotPoseTheta");
    m_logtxReef           = new DoubleLogEntry(log, "/vision/txReef");
    m_logtyReef           = new DoubleLogEntry(log, "/vision/tyReef");
    m_logtidReef          = new IntegerLogEntry(log, "/vision/tidReef");
  }

  // ----- periodic -----
  @Override
  public void periodic() {
    // PeriodicReef in C++
    PeriodicReef();
  }

  // ----- public API (names preserved) -----
  public boolean IsValidReef() { return m_isValidReef; }

  public double GetX() { return m_netBufferAlli[BotPoseIndices.eX]; }

  public double GetY() { return m_netBufferAlli[BotPoseIndices.eY]; }

  public double GetYaw() { return m_netBufferAlli[BotPoseIndices.eYaw]; }

  public double GetLatency() { return m_netBufferAlli[BotPoseIndices.eLatency]; }

  public int GetTagId() { return m_tidReef; }

  public void EnableReefLEDs() {
    // Limelight LED mode via NT: setDouble(3) -> force on
    m_netTableReef.getEntry("ledMode").setDouble(3.0);
  }

  public void DisableReefLEDs() {
    // setDouble(1) -> force off
    m_netTableReef.getEntry("ledMode").setDouble(1.0);
  }

  // ----- private methods -----
  private void PeriodicReef() {
    // Limelight "tv" (target valid) as double; 1.0 = valid
    m_isValidReef = (m_netTableReef.getEntry("tv").getDouble(0.0) == 1.0);
    // C++ used PutNumber with a bool; Java dashboard has a boolean API; both exist
    SmartDashboard.putBoolean("tv Reef", m_isValidReef);

    if (m_isValidReef) {
      // Arrays from LL: use getDoubleArray(defaultArray)
      m_netBufferField = m_netTableReef.getEntry("botpose").getDoubleArray(m_zero_vector);
      m_logRobotPoseX.append(m_netBufferField[BotPoseIndices.eX]);
      m_logRobotPoseY.append(m_netBufferField[BotPoseIndices.eY]);
      m_logRobotPoseTheta.append(m_netBufferField[BotPoseIndices.eYaw]);

      // Alliance-oriented pose
      m_netBufferAlli = m_netTableReef
          .getEntry("botpose_orb_wpiblue")
          .getDoubleArray(m_zero_vector);
      m_logRobotAlliPoseX.append(m_netBufferAlli[BotPoseIndices.eX]);
      m_logRobotAlliPoseY.append(m_netBufferAlli[BotPoseIndices.eY]);
      m_logRobotAlliPoseTheta.append(m_netBufferAlli[BotPoseIndices.eYaw]);
      m_logLL_Latency.append(m_netBufferAlli[BotPoseIndices.eLatency]);

      SmartDashboard.putNumber("xReef",   m_netBufferAlli[BotPoseIndices.eX]);
      SmartDashboard.putNumber("yReef",   m_netBufferAlli[BotPoseIndices.eY]);
      SmartDashboard.putNumber("yawReef", m_netBufferAlli[BotPoseIndices.eYaw]);

      // Individual measurements
      m_tyReef = m_netTableReef.getEntry("ty").getDouble(0.0);
      m_txReef = m_netTableReef.getEntry("tx").getDouble(0.0);
      // Limelight typically publishes tid as a number (double). Cast to int for parity.
      m_tidReef = (int)m_netTableReef.getEntry("tid").getDouble(0.0);

      m_logtxReef.append(m_txReef);
      m_logtyReef.append(m_tyReef);
      m_logtidReef.append(m_tidReef);
    }
  }
}
