package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.DataLog;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.RelativeEncoder;


import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel; // For MotorType

import frc.robot.Constants;

// NOTE: If you already have ELevels defined elsewhere in your project, remove the enum below.
// I include it so this file is self-contained; duplicate definition will produce compile errors
// if your project already defines ELevels.


public class ElevatorSubsystem extends SubsystemBase {
    // --- constants (converted from the header)
    private static final double c_defaultResetTurns = 0.0;
    private static final double c_defaultParkTurns = 0.0;
    private static final double c_defaultL1Turns = 0.0;
    private static final double c_defaultL2Turns = 2.0;
    private static final double c_defaultL3Turns = 15.0;
    private static final double c_defaultL4Turns = 40.0;
    private static final double c_defaultLoadTurns = 10.0;
    private static final double c_algaeRemovalL3_4 = 15.1;
    private static final double c_algaeRemovalL2_3 = 3.5;
    private static final double c_algaeRemovalGripBall = -4.0;
    private static final double c_elevToleranceTurns = 6.0;

    // --- REV / motor config objects (Java API)
    private final SparkFlexConfig m_leadConfig = new SparkFlexConfig();
    private final SparkFlexConfig m_followConfig = new SparkFlexConfig();

    // Use your CAN IDs from ConstantsCANIDs (kept the same names)
    private final SparkFlex m_leadMotor = new SparkFlex(23,
            SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_leadRelativeEnc = m_leadMotor.getEncoder();
    private final SparkClosedLoopController m_leadPIDController = m_leadMotor.getClosedLoopController();

    private final SparkFlex m_followMotor = new SparkFlex(24,
            SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_followRelativeEnc = m_followMotor.getEncoder();
    private final SparkClosedLoopController m_followPIDController = m_followMotor.getClosedLoopController();

    // Limit switches on the lead motor (forward/reverse)
    private final SparkLimitSwitch m_lowerLimit = m_leadMotor.getReverseLimitSwitch();
    private final SparkLimitSwitch m_upperLimit = m_leadMotor.getForwardLimitSwitch();

    // Data logging entry
    private final DoubleLogEntry m_log;

    // directional multipliers & runtime state (kept same names)
    private double m_leadDirection = 1.0;
    private double m_followDirection = 1.0;

    private double m_position = 0.0;
    private ClosedLoopSlot m_slot = ClosedLoopSlot.kSlot0;

    private Constants.OperatorConstants.ELevels m_level = Constants.OperatorConstants.ELevels.L1;

    // periodic counter to mimic C++ static local "count"
    private int m_periodicCounter = 0;

    // Constructor
    public ElevatorSubsystem() {
        // Setup datalog
        DataLog log = DataLogManager.getLog();
        m_log = new DoubleLogEntry(log, "/elevator/position");

        // Configure lead motor
        m_leadConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
                .inverted(true);
        m_leadConfig.closedLoopRampRate(0.0);
        // The C++ used a closedLoop.OutputRange(kMinOut, kMaxOut). In Java use outputRange:
        m_leadConfig.closedLoop.outputRange(-1.0, 1.0);
        // Apply configuration (same ResetMode / PersistMode used in C++ example)
        m_leadMotor.configure(m_leadConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        // Configure follow motor
        m_followConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
                .inverted(false);
        m_followConfig.closedLoopRampRate(0.0);
        m_followConfig.closedLoop.outputRange(-1.0, 1.0);
        m_followMotor.configure(m_followConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        // Reset relative encoders (setPosition)
        m_leadRelativeEnc.setPosition(0.0);
        m_followRelativeEnc.setPosition(0.0);

        // Put initial SmartDashboard values (same keys your C++ used)
        SmartDashboard.putNumber("ElevatorLeadMotorDirection", m_leadDirection);
        SmartDashboard.putNumber("ElevatorFollowMotorDirection", m_followDirection);

        SmartDashboard.putNumber("ElevatorParkTurns", c_defaultParkTurns);
        SmartDashboard.putNumber("ElevatorResetTurns", c_defaultResetTurns);

        // Initialize Preferences keys similar to C++ Preferences::InitDouble(...)
        Preferences.initDouble("ElevatorPosDownP", 0.01);
        Preferences.initDouble("ElevatorPosUpP", 0.1);
        Preferences.initDouble("ElevatorPosI", 0.00003);
        Preferences.initDouble("ElevatorPosD", 0.0);
        Preferences.initDouble("ElevatorPosFF", 0.0);
        Preferences.initDouble("ElevatorL4", c_defaultL4Turns);
        Preferences.initDouble("ElevatorPosRR", 0.0);

        SmartDashboard.putNumber("ElevatorLeadMotorPos", 1.0);
        SmartDashboard.putNumber("ElevatorFollowMotorPos", 1.0);
        SmartDashboard.putNumber("ElevatorGoToRel", 1.0);

        SmartDashboard.putBoolean("L1", false);
        SmartDashboard.putBoolean("L2", false);
        SmartDashboard.putBoolean("L3", false);
        SmartDashboard.putBoolean("L4", false);
    }

    //
    // Keep the original C++-style method names while also overriding WPILib's periodic()
    // so the robot framework calls periodic() and your C++-style Periodic() body is executed.
    //


    // This mirrors your C++ Periodic() method
    @Override
    public void periodic() {
        m_periodicCounter++;
        if (m_periodicCounter % 20 == 0) {
            double lastDownP = 0.0;
            double lastUpP = 0.0;
            double lastI = 0.0;
            double lastD = 0.0;
            double lastFF = 0.0;

            double pDown = Preferences.getDouble("ElevatorPosDownP", 0.01);
            double pUp = Preferences.getDouble("ElevatorPosUpP", 0.1);
            double i = Preferences.getDouble("ElevatorPosI", 0.00003);
            double d = Preferences.getDouble("ElevatorPosD", 0.0);
            double ff = Preferences.getDouble("ElevatorPosFF", 0.0);

            if (pDown != lastDownP) {
                m_leadConfig.closedLoop.p(pDown, ClosedLoopSlot.kSlot0);
                m_followConfig.closedLoop.p(pDown, ClosedLoopSlot.kSlot0);
                m_leadMotor.configure(m_leadConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
                m_followMotor.configure(m_followConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
            }
            if (pUp != lastUpP) {
                m_leadConfig.closedLoop.p(pUp, ClosedLoopSlot.kSlot1);
                m_followConfig.closedLoop.p(pUp, ClosedLoopSlot.kSlot1);
                m_leadMotor.configure(m_leadConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
                m_followMotor.configure(m_followConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
            }
            if (i != lastI) {
                m_leadConfig.closedLoop.i(i, ClosedLoopSlot.kSlot0); // apply to down-slot per original
                m_followConfig.closedLoop.i(i, ClosedLoopSlot.kSlot0);
                m_leadMotor.configure(m_leadConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
                m_followMotor.configure(m_followConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
            }
            if (d != lastD) {
                m_leadConfig.closedLoop.d(d, ClosedLoopSlot.kSlot0).d(d, ClosedLoopSlot.kSlot1);
                m_followConfig.closedLoop.d(d, ClosedLoopSlot.kSlot0).d(d, ClosedLoopSlot.kSlot1);
                m_leadMotor.configure(m_leadConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
                m_followMotor.configure(m_followConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
            }
            if (ff != lastFF) {
                // velocityFF() is available; REV docs mark this deprecated in favor of feedForward
                m_leadConfig.closedLoop.velocityFF(ff, ClosedLoopSlot.kSlot0).velocityFF(ff,
                        ClosedLoopSlot.kSlot1);
                m_followConfig.closedLoop.velocityFF(ff, ClosedLoopSlot.kSlot0).velocityFF(ff,
                        ClosedLoopSlot.kSlot1);
                m_leadMotor.configure(m_leadConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
                m_followMotor.configure(m_followConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters);
            }

            // Save last values so we don't reconfigure repeatedly
            lastDownP = pDown;
            lastUpP = pUp;
            lastI = i;
            lastD = d;
            lastFF = ff;

            // mimic some of the C++ behavior: check for motion zone and switch slot if needed
            double currentPos = m_leadRelativeEnc.getPosition();
            boolean bGoingToBottom = currentPos < 10.0 && m_position < 1.0;
            boolean bGoingToTop = currentPos > 30.0 && m_position > 39.0;
            if ((bGoingToBottom || bGoingToTop) && m_slot == ClosedLoopSlot.kSlot1) {
                m_slot = ClosedLoopSlot.kSlot0;
                m_leadPIDController.setReference(m_position, SparkBase.ControlType.kPosition, m_slot);
                m_followPIDController.setReference(m_position, SparkBase.ControlType.kPosition, m_slot);
            }

            SmartDashboard.putNumber("ElevatorLeadMotorPos Echo", m_leadRelativeEnc.getPosition());
            SmartDashboard.putNumber("ElevatorFollowMotorPos Echo", m_followRelativeEnc.getPosition());
            SmartDashboard.putBoolean("ElevatorUpperLimit", m_upperLimit.isPressed());
            SmartDashboard.putBoolean("ElevatorLowerLimit", m_lowerLimit.isPressed());
        }

        // Append to datalog occasionally (keeps field similar to C++ m_log usage)
        m_log.append(m_leadRelativeEnc.getPosition());
    }

    // Keep your C++-style method signatures (same names)
    public void Stop() {
        m_followMotor.stopMotor();
        m_leadMotor.stopMotor();
    }

    public void GoToPosition(Constants.OperatorConstants.ELevels eLevel) {
        if (eLevel != Constants.OperatorConstants.ELevels.algaeRemovalL3_4) {
            SetPresetLevel(eLevel);
        }

        double transition = GetPositionForLevel(Constants.OperatorConstants.ELevels.L3) + c_elevToleranceTurns;
        if (eLevel == Constants.OperatorConstants.ELevels.L1 && (GetCurrentPosition() > transition)) {
            // Too high, will slam; caller needs to wait and re-go to L1
            eLevel = Constants.OperatorConstants.ELevels.L3;
        }

        GoToPosition(GetPositionForLevel(eLevel));
    }

    public void ElevatorReset() {
        m_leadRelativeEnc.setPosition(0.0);
        m_followRelativeEnc.setPosition(0.0);
    }

    public double GetCurrentPosition() {
        return m_leadRelativeEnc.getPosition();
    }

    public void GotoPositionRel(double relPos) {
        boolean bDown = relPos < 0.0;
        relPos = SmartDashboard.getNumber("ElevatorGoToRel", 1.0);
        relPos = Math.max(-10.0, Math.min(10.0, relPos));
        if (bDown) {
            relPos *= -1.0;
        }
        GoToPosition(m_leadRelativeEnc.getPosition() + relPos);
    }

    public boolean IsAtPosition(Constants.OperatorConstants.ELevels level) {
        double levelPos = GetPositionForLevel(level);
        double difference = Math.abs(m_leadRelativeEnc.getPosition() - levelPos);
        return (difference > -c_elevToleranceTurns) && (difference < c_elevToleranceTurns);
    }

    public boolean GetUpperLimit() {
        return m_upperLimit.isPressed();
    }

    public boolean GetLowerLimit() {
        return m_lowerLimit.isPressed();
    }

    public void SetPresetLevel(Constants.OperatorConstants.ELevels level) {
        m_level = level;
        SmartDashboard.putBoolean("L1", (m_level == Constants.OperatorConstants.ELevels.L1));
        SmartDashboard.putBoolean("L2", (m_level == Constants.OperatorConstants.ELevels.L2));
        SmartDashboard.putBoolean("L3", (m_level == Constants.OperatorConstants.ELevels.L3));
        SmartDashboard.putBoolean("L4", (m_level == Constants.OperatorConstants.ELevels.L4));
    }

    public Constants.OperatorConstants.ELevels GetPresetLevel() {
        return m_level;
    }

    public void GoToPresetLevel() {
        GoToPosition(m_level);
    }

    public double GetPositionForLevel(Constants.OperatorConstants.ELevels eLevel) {
        // mirrors the C++ static table/Preferences logic
        if (eLevel == Constants.OperatorConstants.ELevels.L4) {
            return Preferences.getDouble("ElevatorL4", c_defaultL4Turns);
        } else {
            // index mapping: L1->0, L2->1, L3->2, L4->3
            switch (eLevel) {
                case L1:
                    return c_defaultL1Turns;
                case L2:
                    return c_defaultL2Turns;
                case L3:
                    return c_defaultL3Turns;
                case L4:
                    return c_defaultL4Turns;
                case algaeRemovalL3_4:
                    return c_algaeRemovalL3_4;
                case algaeRemovalL2_3:
                    return c_algaeRemovalL2_3;
                default:
                    return 0.0;
            }
        }
    }

    public void GoToPosition(double position) {
        m_position = Math.max(0.0, Math.min(position, c_defaultL4Turns));

        SmartDashboard.putNumber("ElevatorLeadMotorPos", position);
        SmartDashboard.putNumber("ElevatorFollowMotorPos", position);

        m_slot = ClosedLoopSlot.kSlot1; // up slot by default
        if (m_position < 1.5) {
            m_slot = ClosedLoopSlot.kSlot0; // down slot
        }
        // setSetpoint used in updated REV Java API (setReference is deprecated)
        m_leadPIDController.setReference(m_position, SparkBase.ControlType.kPosition, m_slot);
        m_followPIDController.setReference(m_position, SparkBase.ControlType.kPosition, m_slot);
    }
}
