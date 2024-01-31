package frc.robot;

// Contains all ports on our robot

public class RobotMap {
  public static final int IS_PRACTICE_BOT_DIO = 0;

  public static class mapLEDs {
    public static final int LEDs_CANdle_ID = 60;
  }

  public static class mapControllers {
    public static final int DRIVER_USB = 0;
    public static final int OPERATOR_USB = 1;

  }

  public static class mapClimber {
    public static final int CLIMBER_MOTOR_CAN = 30;
  }

  public static class mapDrivetrain {
    public static final String CAN_BUS_NAME = "Swerve";
    public static final int PIGEON_CAN = 0;

    // Module 0
    public static final int FRONT_LEFT_DRIVE_CAN = 0;
    public static final int FRONT_LEFT_STEER_CAN = 1;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_CAN = 0;

    // Module 1
    public static final int FRONT_RIGHT_DRIVE_CAN = 2;
    public static final int FRONT_RIGHT_STEER_CAN = 3;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_CAN = 1;

    // Module 2
    public static final int BACK_LEFT_DRIVE_CAN = 4;
    public static final int BACK_LEFT_STEER_CAN = 5;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_CAN = 2;

    // Module 3
    public static final int BACK_RIGHT_DRIVE_CAN = 6;
    public static final int BACK_RIGHT_STEER_CAN = 7;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_CAN = 3;
  }

  public static class mapIntake {
    public static final int INTAKE_ROLLER_MOTOR_CAN = 20;
    public static final int INTAKE_LEFT_CENTERING_MOTOR_CAN = 21;
    public static final int INTAKE_RIGHT_CENTERING_MOTOR_CAN = 22;
  }

  public static class mapShooter {
    public static final int SHOOTER_LEFT_MOTOR_CAN = 10;
    public static final int SHOOTER_RIGHT_MOTOR_CAN = 11;
    public static final int SHOOTER_PITCH_MOTOR_CAN = 12;
  }

  public static class mapTurret {
    public static final int TURRET_MOTOR_CAN = 40;
    public static final int TURRET_ABSOLUTE_ENCODER_DIO = 1;
  }

  public static class mapVision {
  }
}
