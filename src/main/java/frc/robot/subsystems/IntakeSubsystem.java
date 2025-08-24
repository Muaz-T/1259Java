package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Java translation of your C++ IntakeSubsystem that used rev::spark::SparkMax APIs.
 *
 * I kept the same method & variable names (Periodic, Set, AlignIntake, ParkIntakeForLoad, etc.)
 * and used the newer REVLib Java API surface (SparkMax, SparkBaseConfig, SparkClosedLoopController, RelativeEncoder).
 */
public class IntakeSubsystem extends SubsystemBase {
    // --- constants ported from the C++ header / cpp ---
    public static final double kIngestSpeed = 1.0;
    public static final double kReleaseSpeed = -1.0;

    public static final double c_InitalDeployPosition = 15.0;
    public static final double c_LoadCoralPosition = 9.238;
    public static final double c_ParkForClimbPosition = 25.0;

    // default PID values (from your cpp)
    private static final double c_defaultIntakeP = 0.03;
    private static final double c_defaultIntakeI = 0.0;
    private static final double c_defaultIntakeD = 0.0;

    // Closed loop slot used in C++
    private static final ClosedLoopSlot c_intakeGeneralPIDSlot = ClosedLoopSlot.kSlot0;

    // --- members (names preserved) ---
    private final Timer m_timer = new Timer();

    private final SparkMax m_deployMotor;
    private final RelativeEncoder m_deployRelativeEnc;
    private final SparkClosedLoopController m_deployPIDController;

    // keep last seen PID values to avoid reconfiguring unnecessarily (C++ used static locals)
    private double lastP = 0.0;
    private double lastI = 0.0;
    private double lastD = 0.0;

    public IntakeSubsystem() {
        // construct SparkMax using the CAN ID constant from your project
        // (I kept the same constant name you used in C++: kIntakeChuteCANID).
        m_deployMotor = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);

        // Build a configuration object similar to the C++ SparkBaseConfig usage
        SparkMaxConfig config = new SparkMaxConfig();
        // set idle mode to brake and not inverted
        config.idleMode(SparkBaseConfig.IdleMode.kBrake)
              .inverted(false);

        // closed-loop ramp rate
        config.closedLoopRampRate(0.0);

        // set closed-loop output range (matching your config.closedLoop.OutputRange(kMinOut, kMaxOut))
        // note: kMinOut/kMaxOut assumed to be in your Constants class (same names as C++)
        config.closedLoop.outputRange(-1.0, 1.0);

        // Apply the configuration to the Spark (no-reset safe params, persist parameters) - same semantics as C++
        m_deployMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // get the relative encoder & closed-loop controller objects from the Spark
        m_deployRelativeEnc = m_deployMotor.getEncoder();
        m_deployPIDController = m_deployMotor.getClosedLoopController();

        // align encoder position with C++'s SetPosition(0.0)
        m_deployRelativeEnc.setPosition(0.0);

        // Initialize Preferences (same keys as your C++ code)
        Preferences.initDouble("kIntakeDeployP", c_defaultIntakeP);
        Preferences.initDouble("kIntakeDeployI", c_defaultIntakeI);
        Preferences.initDouble("kIntakeDeployD", c_defaultIntakeD);

        // Dashboard initial values (same keys as C++)
        SmartDashboard.putNumber("IntakeRel", 1.0);
        SmartDashboard.putNumber("ParkForClimb", c_ParkForClimbPosition);
    }

    /// Will be called periodically whenever the CommandScheduler runs.
    @Override
    public void periodic() {
        LoadDeployPid();
        SmartDashboard.putNumber("Deploy echo", m_deployRelativeEnc.getPosition());
    }

    /// Drives the intake at a given speed
    /// \param speed Desired motor speed to run, ranging from [-1, 1]
    public void Set(double speed) {
        // mirror of commented-out C++ percent output call; use SparkMax.set(double)
        // m_deployMotor.set(speed);
    }

    /// Extends the intake past the pin pop
    public void AlignIntake() {
        // use SparkClosedLoopController.setSetpoint(...) (setReference in older API)
        m_deployPIDController.setReference(c_InitalDeployPosition, SparkBase.ControlType.kPosition, c_intakeGeneralPIDSlot);
    }

    /// Sets the intake position for loading coral
    public void ParkIntakeForLoad() {
        m_deployPIDController.setReference(c_LoadCoralPosition, SparkBase.ControlType.kPosition, c_intakeGeneralPIDSlot);
    }

    /// Sets the intake out of the way for climb
    public void ParkIntakeForClimb() {
        double pos = SmartDashboard.getNumber("ParkForClimb", c_ParkForClimbPosition);
        m_deployPIDController.setReference(pos, SparkBase.ControlType.kPosition, c_intakeGeneralPIDSlot);
    }

    public void GoToPosition(double turns) {
        m_deployPIDController.setReference(turns, SparkBase.ControlType.kPosition, c_intakeGeneralPIDSlot);
    }

    public void GoToPositionRel(double relPos) {
        // read the dashboard key the same way your C++ did
        relPos = SmartDashboard.getNumber("IntakeRel", 1.0);
        m_deployPIDController.setReference(m_deployRelativeEnc.getPosition() + relPos, SparkBase.ControlType.kPosition, c_intakeGeneralPIDSlot);
    }

    public double GetPosition() {
        return m_deployRelativeEnc.getPosition();
    }

    public void Stop() {
        Set(0.0);
    }

    // --- private helpers ---

    // Loads PID constants from Preferences and only reconfigures the Spark when they change
    private void LoadDeployPid() {
        double p = Preferences.getDouble("kIntakeDeployP", c_defaultIntakeP);
        double i = Preferences.getDouble("kIntakeDeployI", c_defaultIntakeI);
        double d = Preferences.getDouble("kIntakeDeployD", c_defaultIntakeD);

        // If P changed, reconfigure only that PID slot's P value
        if (p != lastP) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.p(p, c_intakeGeneralPIDSlot);
            config.inverted(false);
            m_deployMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        }

        if (i != lastI) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.i(i, c_intakeGeneralPIDSlot);
            config.inverted(false);
            m_deployMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        }

        if (d != lastD) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.d(d, c_intakeGeneralPIDSlot);
            config.inverted(false);
            m_deployMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        }
        lastP = p;
        lastI = i;
        lastD = d;
    }
}
