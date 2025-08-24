// LEDSubsystem.java
// NOTE: keep your package as appropriate, e.g. package frc.robot.subsystems;
package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// If your IDs live in a Constants-like class, import that.
// This keeps the original symbol name kLEDCANID used in C++.

public class LEDSubsystem extends SubsystemBase {

  // ===== direct translation of the RBGAColor struct =====
  public static class RBGAColor {
    public int red = 0;
    public int green = 0;
    public int blue = 0;
    public int white = 0;

    public RBGAColor() {}
    public RBGAColor(int r, int g, int b, int w) {
      this.red = r; this.green = g; this.blue = b; this.white = w;
    }
  }

  // ===== C++ file-scope constants -> Java static finals =====
  //                   red  green blue white
  public static final RBGAColor c_colorPink   = new RBGAColor( 80, 10,  15,  0);
  public static final RBGAColor c_colorGreen  = new RBGAColor( 13, 80,   0,  0);
  public static final RBGAColor c_colorBlue   = new RBGAColor(  0,  0, 255,  0);
  public static final RBGAColor c_colorRed    = new RBGAColor(255,  0,   0,  0);
  public static final RBGAColor c_colorOrange = new RBGAColor( 43,  6,   0, 255);
  public static final RBGAColor c_colorYellow = new RBGAColor(255,  0,   0,  0);
  public static final RBGAColor c_colorPurple = new RBGAColor( 80,  0,  80,  0);
  public static final RBGAColor c_colorBlack  = new RBGAColor(  0,  0,   0,  0);
  public static final RBGAColor c_colorWhite  = new RBGAColor(255,255, 255, 10);

  public static final RBGAColor c_defaultColor = c_colorWhite;

  // ===== enums preserved exactly =====
  public enum EAnimation {
    kDefaultAnim,
    kSolid /* = kDefaultAnim */,
    kFade,
    kFlow,
    kStrobe,
    kScanner,
    kBlank
  }

  public enum ECurrentAction {
    kDefaultAction,
    kIdle /* = kDefaultAction */,
    kIntaking,
    kHasCoral,
    kTagVisible,
    kPreCoral,
    kPostCoral,
    kElevator,
    kClimbing,
    kSequenceStart,
    kFollowPath,
    kClimbFinish
  }

  // ===== members =====
  private final CANdleConfiguration m_candleConfig = new CANdleConfiguration();
  private DoubleLogEntry m_log;
  private final CANdle m_candle = new CANdle(1);
  private Optional<Alliance> m_alliance = Optional.empty();

  private boolean m_bAllianceSet = false;

  private static final double c_defaultSpeed = 0.5;
  private static final int c_ledNum = 16;
  private static final int c_ledOffset = 8;

  // Animations (same ctor semantics as C++ version)
  private final ColorFlowAnimation m_colorFlowAnimation =
      new ColorFlowAnimation(0, 0, 0, 0, c_defaultSpeed, c_ledNum,
          ColorFlowAnimation.Direction.Forward, c_ledOffset);

  private final SingleFadeAnimation m_singleFadeAnimation =
      new SingleFadeAnimation(0, 0, 0, 0, c_defaultSpeed, c_ledNum, c_ledOffset);

  private final StrobeAnimation m_strobeAnimation =
      new StrobeAnimation(0, 0, 0, 0, c_defaultSpeed, c_ledNum, c_ledOffset);

  private final LarsonAnimation m_larsonAnimation =
      new LarsonAnimation(0, 0, 0, 0, c_defaultSpeed, c_ledNum,
          LarsonAnimation.BounceMode.Front, 3, c_ledOffset);

  private RBGAColor      m_defaultColor = c_defaultColor;
  private RBGAColor      m_currentColor = c_colorBlack;
  private ECurrentAction m_currentAction = ECurrentAction.kIdle;

  // ===== ctor =====
  public LEDSubsystem() {
    // DataLog setup (WPILib Java)
    final DataLog log = DataLogManager.getLog();
    m_log = new DoubleLogEntry(log, "/subsystem/led");

    // Phoenix CANdle configuration (Java API: configAllSettings, LEDStripType, brightnessScalar)
    m_candleConfig.stripType = LEDStripType.RGB;
    m_candleConfig.brightnessScalar = 1.0;
    m_candle.configAllSettings(m_candleConfig);
    m_candle.clearAnimation(0);

    SetColor(c_colorOrange);

    // Optional error/fault calls can be added if you want parity with C++ debug:
    // m_candle.getLastError();
    // final CANdleFaults faults = new CANdleFaults();
    // m_candle.getFaults(faults);
  }

  // ===== periodic logic preserved =====
  @Override
  public void periodic() {
    if (!m_bAllianceSet) {
      m_bAllianceSet = true;
      m_alliance = DriverStation.getAlliance();
    }

    boolean robotEnabled = SmartDashboard.getBoolean("Robot Enabled", false);
    if (robotEnabled) {
      if (!IsRobotBusy()) {
        SetAnimation(GetDefaultColor(), EAnimation.kSolid);
      }
    } else {
      // Robot disabled -> alliance color flow
      if (m_alliance.isPresent() && m_alliance.get() == Alliance.Blue) {
        if (!IsSameColor(m_currentColor, c_colorBlue)) {
          SetAnimation(c_colorBlue, EAnimation.kFlow);
        }
      } else {
        if (!IsSameColor(m_currentColor, c_colorRed)) {
          SetAnimation(c_colorRed, EAnimation.kFlow);
        }
      }
    }

    // Example log append point:
    // m_log.append(<your double value>);
  }

  // ===== public API (names preserved) =====
  public void SetAnimation(RBGAColor color, EAnimation animate) {
    switch (animate) {
      case kSolid:
        m_candle.clearAnimation(0);
        SetColor(color);
        break;

      case kFade:
        SetColor(m_singleFadeAnimation, color);
        m_candle.animate(m_singleFadeAnimation);
        break;

      case kFlow:
        SetColor(m_colorFlowAnimation, color);
        m_candle.animate(m_colorFlowAnimation);
        break;

      case kStrobe:
        SetColor(m_strobeAnimation, color);
        m_candle.animate(m_strobeAnimation);
        break;

      case kScanner:
        SetColor(m_larsonAnimation, color);
        m_candle.animate(m_larsonAnimation);
        break;

      default: // kBlank or kDefaultAnim -> clear and set solid color
        m_candle.clearAnimation(0);
        SetColor(color);
        break;
    }
  }

  public boolean IsRobotBusy() {
    return m_currentAction != ECurrentAction.kIdle;
  }

  public RBGAColor GetDefaultColor() {
    return m_defaultColor;
  }

  public void SetCurrentAction(ECurrentAction action) {
    m_currentAction = action;
  }

  public ECurrentAction GetCurrentAction() {
    return m_currentAction;
  }

  // ===== private helpers (names/behavior preserved) =====
  private void SetColor(final RBGAColor color) {
    m_currentColor = color;
    // CTRE Java: setLEDs(r,g,b,w,start,count)
    m_candle.setLEDs(color.red, color.green, color.blue, color.white, c_ledOffset, c_ledNum);
  }

  // Overloads to match C++ convenience with BaseTwoSizeAnimation.
  // In Java, each animation exposes setR/setG/setB/setW directly. (Phoenix Java API)
  private void SetColor(final ColorFlowAnimation animation, final RBGAColor color) {
    m_currentColor = color;
    animation.setR(color.red);
    animation.setG(color.green);
    animation.setB(color.blue);
    animation.setW(color.white);
  }

  private void SetColor(final SingleFadeAnimation animation, final RBGAColor color) {
    m_currentColor = color;
    animation.setR(color.red);
    animation.setG(color.green);
    animation.setB(color.blue);
    animation.setW(color.white);
  }

  private void SetColor(final StrobeAnimation animation, final RBGAColor color) {
    m_currentColor = color;
    animation.setR(color.red);
    animation.setG(color.green);
    animation.setB(color.blue);
    animation.setW(color.white);
  }

  private void SetColor(final LarsonAnimation animation, final RBGAColor color) {
    m_currentColor = color;
    animation.setR(color.red);
    animation.setG(color.green);
    animation.setB(color.blue);
    animation.setW(color.white);
  }

  private boolean IsSameColor(final RBGAColor c1, final RBGAColor c2) {
    return c1.red == c2.red && c1.green == c2.green && c1.blue == c2.blue && c1.white == c2.white;
  }
}
