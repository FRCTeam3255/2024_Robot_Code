package frc.robot;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;

/*
 * | Unit Type | Unit to Use |
 * | ---------- | ------------ |
 * | Distance | Meters |
 * | Distance per Time | Meters per Second |
 * | Angle | Degrees |
 * | Angle per Time | Degrees per Second |
 * | Time | Seconds |
 * 
 * If the unit does not fall under any of these types, 
 * add a JavaDoc for that variable specifying it's unit. 
 * Avoid specifying units in the variable name.
 * Preferences that obviously don't use the above units (ex. PID)
 * are exempt from this
 */
public class RobotPreferences {
  public static final class prefDrivetrain {
    // This PID is implemented on each module, not the Drivetrain subsystem.
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0.0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 1);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 1);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0.0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 0.0);

    public static final SN_DoublePreference driveKs = new SN_DoublePreference("driveKs", 0);
    public static final SN_DoublePreference driveKa = new SN_DoublePreference("driveKa", 0);
    public static final SN_DoublePreference driveKv = new SN_DoublePreference("driveKv", 0);

    // This PID is implemented on the Drivetrain subsystem
    public static final SN_DoublePreference autoDriveP = new SN_DoublePreference("autoDriveP", 2);
    public static final SN_DoublePreference autoDriveI = new SN_DoublePreference("autoDriveI", 0);
    public static final SN_DoublePreference autoDriveD = new SN_DoublePreference("autoDriveD", 0);

    public static final SN_DoublePreference autoSteerP = new SN_DoublePreference("autoSteerP", 0.5);
    public static final SN_DoublePreference autoSteerI = new SN_DoublePreference("autoSteerI", 0.0);
    public static final SN_DoublePreference autoSteerD = new SN_DoublePreference("autoSteerD", 0.0);

    /**
     * <b>Units:</b> Percentage from 0 to 1
     */
    public static final SN_DoublePreference minimumSteerSpeedPercent = new SN_DoublePreference(
        "minimumSteerSpeedPercent", 0.01);

    /**
     * <p>
     * Translational speed while manually driving
     * </p>
     * <b>Units:</b> Meters Per Second
     */
    public static final SN_DoublePreference driveSpeed = new SN_DoublePreference("driveSpeed",
        Units.feetToMeters(16.5));

    /**
     * Rotational speed while manually driving
     * MAX: 943.751 DPS (Due to gearing and robot size)
     * <b>Units:</b> Degrees per second
     */
    public static final SN_DoublePreference turnSpeed = new SN_DoublePreference("turnSpeed", 360);

    /**
     * <b>Units:</b> Feet
     */
    public static final SN_DoublePreference autoMaxSpeed = new SN_DoublePreference(
        "autoMaxSpeed", 8);
    /**
     * <b>Units:</b> Feet
     */
    public static final SN_DoublePreference autoMaxAccel = new SN_DoublePreference(
        "autoMaxAccel", 6);

    /**
     * Pose estimator standard deviation for encoder & gyro data
     * <b>Units:</b> Feet
     */
    public static final SN_DoublePreference measurementStdDevsPosition = new SN_DoublePreference(
        "measurementStdDevsPosition", Units.metersToFeet(0.1));
    /**
     * Pose estimator standard deviation for encoder & gyro data
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference measurementStdDevsHeading = new SN_DoublePreference(
        "measurementStdDevsHeading", Units.metersToFeet(0.1));
  }

  public static final class prefVision {
    /**
     * Pose estimator standard deviation for vision data
     * <b>Units:</b> Feet
     */
    public static final SN_DoublePreference visionStdDevsPosition = new SN_DoublePreference(
        "visionStdDevsPosition", Units.metersToFeet(0.9));
    /**
     * Pose estimator standard deviation for vision data
     * <b>Units:</b> Degrees
     */
    public static final SN_DoublePreference visionStdDevsHeading = new SN_DoublePreference(
        "visionStdDevsHeading", Units.metersToFeet(0.9));

  }

  public static final class prefShooter {
    public static final SN_DoublePreference leftShooterS = new SN_DoublePreference("leftShooterS", 0);
    public static final SN_DoublePreference leftShooterV = new SN_DoublePreference("leftShooterV", 0);
    public static final SN_DoublePreference leftShooterP = new SN_DoublePreference("leftShooterP", 0);
    public static final SN_DoublePreference leftShooterI = new SN_DoublePreference("leftShooterI", 0);
    public static final SN_DoublePreference leftShooterD = new SN_DoublePreference("leftShooterD", 0);

    public static final SN_DoublePreference rightShooterS = new SN_DoublePreference("rightShooterS", 0);
    public static final SN_DoublePreference rightShooterV = new SN_DoublePreference("rightShooterV", 0);
    public static final SN_DoublePreference rightShooterP = new SN_DoublePreference("rightShooterP", 0);
    public static final SN_DoublePreference rightShooterI = new SN_DoublePreference("rightShooterI", 0);
    public static final SN_DoublePreference rightShooterD = new SN_DoublePreference("rightShooterD", 0);

    public static final SN_DoublePreference shooterVelocityVoltage = new SN_DoublePreference("shooterVelocityVoltage",
        0);

    public static final SN_DoublePreference leftShooterVelocity = new SN_DoublePreference("leftShooterVelocity", 1);
    public static final SN_DoublePreference leftShooterFeedForward = new SN_DoublePreference("leftShooterFeedForward",
        0);
    public static final SN_DoublePreference rightShooterVelocity = new SN_DoublePreference("rightShooterVelocity", 1);
    public static final SN_DoublePreference rightShooterFeedForward = new SN_DoublePreference("rightShooterFeedForward",
        0);

  }
}
