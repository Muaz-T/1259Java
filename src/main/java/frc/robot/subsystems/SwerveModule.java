package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.Math; // For constants like Math.PI (replacement for <numbers>)
import java.lang.String; // Not strictly necessary; java.lang is imported automatically

// SmartDashboard and Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

// For <cstdlib> — Java equivalents are in java.util and java.lang
import java.util.Random;
import java.lang.System;

import edu.wpi.first.util.datalog.DataLog;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager;

import edu.wpi.first.networktables.NetworkTableEntry;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.BaseUnits;

public class SwerveModule extends SubsystemBase {

    private TalonFX m_driveMotor;
    private SparkFlex m_turningMotor;
    boolean m_driveMotorReversed;
    private AnalogInput m_absEnc;
    double m_offset;
    SparkFlexConfig m_turningConfig = new SparkFlexConfig();
    RelativeEncoder m_turningEncoder;
    CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
    Slot0Configs slot0Configs = new Slot0Configs();
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
    Timer m_Timer = new Timer();
    double m_turnP = 0.03;
    double m_turnI = 0.0;
    double m_turnD = 0.0;
    SparkClosedLoopController m_turningPIDController;
    double m_currentMaxSpeed = 1.0;

    public SwerveModule(final int driveMotorCanId, final int turningMotorCanId, double offset, boolean driveMotorReversed) {
        m_driveMotor = new TalonFX(driveMotorCanId);
        m_turningMotor = new SparkFlex(turningMotorCanId, MotorType.kBrushless);
        m_driveMotorReversed = driveMotorReversed;
        m_absEnc = new AnalogInput((turningMotorCanId / 2) - 1);
        m_offset = offset;
        m_turningPIDController = m_turningMotor.getClosedLoopController();

        m_turningConfig.encoder.positionConversionFactor(2.0 * Math.PI);
        m_turningConfig.inverted(driveMotorReversed);

        double initPosition = -25.0 * voltageToRadians(m_absEnc.getVoltage());
        m_turningEncoder = m_turningMotor.getEncoder();
        m_turningEncoder.setPosition(initPosition); 

        currentLimitConfigs.withStatorCurrentLimit(60.0);
        currentLimitConfigs.withStatorCurrentLimitEnable(true);
        currentLimitConfigs.withSupplyCurrentLimit(60.0);
        currentLimitConfigs.withSupplyCurrentLimitEnable(true);
        currentLimitConfigs.withSupplyCurrentLowerLimit(70.0); //Will do nothing right now, larger than SupplyCurrentLimit
        currentLimitConfigs.withSupplyCurrentLowerTime(0.85);

        m_driveMotor.setPosition(0.0);

        double kDriveP = 0.0025; // 0.1;
        double kDriveI = 0;
        double kDriveD = 0;
        // constexpr double kDriveFF = 0.055;//0.047619;

        double m_max = 1.0;
        double m_min = -1.0;

        slot0Configs.withKP(kDriveP);
        slot0Configs.withKI(kDriveI);
        slot0Configs.withKD(kDriveD);

        motorOutputConfigs.withPeakForwardDutyCycle(m_max);
        motorOutputConfigs.withPeakReverseDutyCycle(m_min);
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);

        m_driveMotor.getConfigurator().apply(currentLimitConfigs);
        m_driveMotor.getConfigurator().apply(slot0Configs);
        m_driveMotor.getConfigurator().apply(motorOutputConfigs);

        m_turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        m_turningConfig.closedLoop.outputRange(-1.0, 1.0);

        SmartDashboard.putNumber("SwrvP", m_turnP);
        SmartDashboard.putNumber("SwrvI", m_turnI);
        SmartDashboard.putNumber("SwrvD", m_turnD);

        m_turningConfig.closedLoop.pid(m_turnP, m_turnI, m_turnD);
        m_turningConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);

        m_turningMotor.configure(m_turningConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        
        m_Timer.start();
        m_Timer.reset();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

      double turnP = SmartDashboard.getNumber("SwrvP", m_turnP);
      double turnI = SmartDashboard.getNumber("SwrvI", m_turnI);
      double turnD = SmartDashboard.getNumber("SwrvD", m_turnD);

      boolean updateConfig = false;

      if (turnP != m_turnP)
      {
        m_turnP = turnP;
        m_turningConfig.closedLoop.p(m_turnP);
        updateConfig = true;
      }
      if (turnI != m_turnI)
      {
        m_turnI = turnI;
        m_turningConfig.closedLoop.i(m_turnI);
        updateConfig = true;
      }
      if (turnD != m_turnD)
      {
        m_turnD = turnD;
        m_turningConfig.closedLoop.d(m_turnD);
        updateConfig = true;
      }
      if (updateConfig)
      {
        m_turningMotor.configure(m_turningConfig, SparkFlex.ResetMode.kNoResetSafeParameters, SparkFlex.PersistMode.kPersistParameters); // TODO: VERIFY CONFIGURATION
      }

    }

    public void resyncAbsRelEnc(){
        double angleInRad = -25.0 * voltageToRadians(m_absEnc.getVoltage());         // Returns rotations between 0 and 1
        m_turningEncoder.setPosition(angleInRad);
    }

    public SwerveModuleState getState(){
        Rotation2d swerveRot = new Rotation2d(getTurnPosition());
        SwerveModuleState swerveState = new SwerveModuleState(calcMetersPerSec(), swerveRot);
        return swerveState;
    }

    public double getTurnPosition() {
      // Negative sign is because turning motor rotates opposite wheel
      return (m_turningEncoder.getPosition() / -25.0);
    }

    public double calcMetersPerSec(){
      // We multiply by -1 if the motor is reversed so that velocity is calculated in the same direction
      double reverse = m_driveMotorReversed ? 1.0 : -1.0;

      return reverse * (0.1016* Math.PI) * m_driveMotor.getVelocity().getValue().in(RotationsPerSecond) / 6.23;
    }

    public SwerveModulePosition GetPosition(){
      Rotation2d swerveRot = new Rotation2d(getTurnPosition());
      SwerveModulePosition swervePosition = new SwerveModulePosition(CalcMeters(), swerveRot);
      return swervePosition;
    }
    
    public double CalcMeters()
    {
      // We multiply by -1 if the motor is reversed so that distance is calculated in the same direction
      double reverse = m_driveMotorReversed ? 1.0 : -1.0;

      return reverse * (0.1016 * Math.PI) * m_driveMotor.getPosition().getValue().in(Rotations) / 6.23;
    }

    public void SetDesiredState(SwerveModuleState referenceState)
    {
      // Optimize the reference state to avoid spinning further than 90 degrees
      double currPosition = getTurnPosition();
      Rotation2d currRotation = new Rotation2d(currPosition);
      referenceState.optimize(currRotation);
    
      if (referenceState.speedMetersPerSecond != 0.0)
      {
        double speed = referenceState.speedMetersPerSecond;
        speed *= m_driveMotorReversed ? -1.0 : 1.0;
        m_driveMotor.set(speed);  
      }
      else
      {
        m_driveMotor.set(0.0);
      }
    
      // Calculate the turning motor output from the turning PID controller.
      // Negative sign is because turning motor rotates opposite wheel
      double newRef = -25.0 * referenceState.angle.getRadians();
    
      //frc::SmartDashboard::PutNumber("Turn Ref Motor" + m_id, newRef);
      m_turningPIDController.setReference(newRef, SparkBase.ControlType.kPosition);
    }

    public TalonFX getTalon() {
      return m_driveMotor;
    }

    public double voltageToRadians(double voltage) {    
        double angle = voltage * (2.0 * Math.PI / 4.78);
        angle -= m_offset;
    
        // Java's fmod equivalent: Math.floorMod doesn't work for doubles, so use remainder and adjust
        angle = (angle + 2 * Math.PI) % (2 * Math.PI);
    
        return angle;
    }
    
    public void SetMaxSpeed(double newMaxSpeed) {m_currentMaxSpeed = newMaxSpeed;}

 }
