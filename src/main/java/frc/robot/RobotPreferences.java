package frc.robot;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;

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

    public static final SN_DoublePreference minimumSteerSpeedPercent = new SN_DoublePreference(
        "minimumSteerSpeed", 0.01);

    // Translational speed (feet per second) while manually driving
    // MAX: 16.3 FPS (Due to gearing)
    public static final SN_DoublePreference driveSpeed = new SN_DoublePreference("driveSpeed", 16.5);

    // Rotational speed (degrees per second) while manually driving
    // MAX: 943.751 DPS (Due to gearing and robot size)
    public static final SN_DoublePreference turnSpeed = new SN_DoublePreference("turnSpeed", 360);

    public static final SN_DoublePreference autoMaxSpeedFeet = new SN_DoublePreference(
        "autoMaxSpeedFeet", 8);
    public static final SN_DoublePreference autoMaxAccelFeet = new SN_DoublePreference(
        "autoMaxAccelFeet", 6);

    // Pose estimator standard deviations for encoder & gyro data
    public static final SN_DoublePreference measurementStdDevsFeet = new SN_DoublePreference(
        "measurementStdDevsFeet", Units.metersToFeet(0.1));
    public static final SN_DoublePreference measurementStdDevsDegrees = new SN_DoublePreference(
        "measurementStdDevsDegrees", Units.metersToFeet(0.1));
  }

  public static final class prefVision {
    // Pose estimator standard deviations for vision data
    public static final SN_DoublePreference measurementStdDevsFeet = new SN_DoublePreference(
        "measurementStdDevsFeet", Units.metersToFeet(0.9));
    public static final SN_DoublePreference measurementStdDevsDegrees = new SN_DoublePreference(
        "measurementStdDevsDegrees", Units.metersToFeet(0.9));

  }
}
