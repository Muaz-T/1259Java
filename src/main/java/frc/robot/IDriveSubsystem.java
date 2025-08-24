package frc.robot;  // Use your actual package

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a swerve drive style DriveSubsystem.
 */
public interface IDriveSubsystem {

    /**
     * Drive the robot.
     * @param xSpeed meters per second in the x direction
     * @param ySpeed meters per second in the y direction
     * @param rot radians per second rotation rate
     * @param fieldRelative true if driving relative to the field, false if relative to robot
     */
    void Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative);

    /**
     * Drive with rotation inputs as xRot and yRot (method semantics per your C++ code).
     * @param xSpeed meters per second in the x direction
     * @param ySpeed meters per second in the y direction
     * @param xRot double (unitless, likely joystick input)
     * @param yRot double (unitless)
     * @param fieldRelative true if field-relative control
     */
    void RotationDrive(double xSpeed, double ySpeed, double xRot, double yRot, boolean fieldRelative);

    /** Update the odometry calculation. */
    void UpdateOdometry();

    /**
     * Reset odometry to a specific pose.
     * @param pose Pose2d to reset to
     */
    void ResetOdometry(Pose2d pose);

    // Commented out in original, omit or add if needed
    // void SetHeading(double headingDegrees);

    /**
     * Get the robot's pitch in degrees.
     * @return pitch in degrees
     */
    double GetPitch();

    /**
     * Get the robot's current pose on the field.
     * @return Pose2d representing robot position and rotation
     */
    Pose2d GetPose();

    /**
     * Get the chassis speeds.
     * @return ChassisSpeeds object representing current speeds
     */
    ChassisSpeeds GetChassisSpeeds();

    /**
     * Set the desired states for the swerve modules.
     * @param desiredStates array of 4 SwerveModuleState objects
     */
    void SetModuleStates(SwerveModuleState[] desiredStates);

    /**
     * Get the robot's X position in meters.
     * @return X position in meters
     */
    double GetX();

    /**
     * Get the robot's Y position in meters.
     * @return Y position in meters
     */
    double GetY();

    /** Resynchronize absolute and relative encoders. */
    void ResyncAbsRelEnc();

    /**
     * Override Xbox controller input.
     * @param bOverride true to override
     */
    void SetOverrideXboxInput(boolean bOverride);

    /** Set wheels to face forward. */
    void WheelsForward();

    /** Set wheels to face left. */
    void WheelsLeft();

    /** Set wheels to face backward. */
    void WheelsBackward();

    /** Set wheels to face right. */
    void WheelsRight();

    /** Toggle slow speed mode. */
    void ToggleSlowSpeed();
}
