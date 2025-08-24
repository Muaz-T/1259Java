package frc.robot;  // Replace with your actual package

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PigeonGyro {
    private final boolean USE_PIGEON_2 = true;

    // Member variable same name as C++: m_gyro
    private Pigeon2 m_gyro;
    // If you want to support old PigeonIMU, add it here (commented out)
    // private PigeonIMU m_gyro;

    // Constructor
    public PigeonGyro(int CANId) {
        if (USE_PIGEON_2) {
            m_gyro = new Pigeon2(CANId);
        } else {
            // m_gyro = new PigeonIMU(CANId);
        }
    }

    public Rotation2d GetRotation2d() {
        double retVal;
        if (USE_PIGEON_2) {
            retVal = Math.IEEEremainder(m_gyro.getYaw().getValueAsDouble(), 360.0);
        } else {
            // retVal = Math.IEEEremainder(m_gyro.getFusedHeading(), 360.0);
            retVal = 0.0; // placeholder
        }
        if (retVal > 180.0) {
            retVal -= 360.0;
        }
        return Rotation2d.fromDegrees(retVal);
    }

    public double GetYaw() {
        if (USE_PIGEON_2) {
            return m_gyro.getYaw().getValueAsDouble();
        } else {
            // return m_gyro.getFusedHeading();
            return 0.0;
        }
    }

    public double GetRoll() {
        if (USE_PIGEON_2) {
            return m_gyro.getRoll().getValueAsDouble();
        } else {
            return 0.0;
        }
    }

    public double GetPitch() {
        if (USE_PIGEON_2) {
            return m_gyro.getPitch().getValueAsDouble();
        } else {
            return 0.0;
        }
    }

    public void Set(double yaw) {
        if (USE_PIGEON_2) {
            m_gyro.setYaw(yaw);
        } else {
            // m_gyro.setFusedHeading(yaw);
        }
    }

    public double GetTurnRate() {
        if (USE_PIGEON_2) {
            return m_gyro.getAngularVelocityZWorld().getValueAsDouble();
        } else {
            // double[] turnRates = new double[3];
            // m_gyro.getRawGyro(turnRates);
            // return turnRates[2];
            return 0.0;
        }
    }

    // Optional Reset method (commented out in C++)
    /*
    public void Reset() {
        if (USE_PIGEON_2) {
            m_gyro.setYaw(0.0);
        } else {
            // m_gyro.setFusedHeading(0.0);
        }
    }
    */
}
