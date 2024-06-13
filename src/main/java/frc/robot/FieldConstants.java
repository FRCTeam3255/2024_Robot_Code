package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Contains all constant poses for the field as well as a method to obtain those
 * constants, depending on alliance
 */
public class FieldConstants {
  public static Optional<Alliance> ALLIANCE = Optional.empty();

  /**
   * <b> Units: </b> Meters
   */
  // public static double FIELD_LENGTH = 12.85875;
  public static double FIELD_LENGTH = 16.541;

  /**
   * Gets the positions of all of the necessary field elements on the field. All
   * coordinates are in meters and are relative to the blue alliance. See
   * <a href=
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
   * Robot Coordinate Systems</a>
   * 
   * @return An array of field element positions. Your Speaker, Amp, Source, Left
   *         Stage, Center Stage, Right Stage, Subwoofer, Shuffle
   */
  public static Supplier<Pose3d[]> GET_FIELD_POSITIONS(boolean fairbotics) {
    if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
      if (fairbotics) {
        return () -> new Pose3d[] { redConstants.FAIRBOTICS_SPEAKER, redConstants.AMP, redConstants.SOURCE,
            redConstants.LEFT_STAGE,
            redConstants.CENTER_STAGE, redConstants.RIGHT_STAGE, redConstants.SUBWOOFER, redConstants.SHUFFLE };

      }
      return () -> new Pose3d[] { redConstants.SPEAKER_CENTER, redConstants.AMP, redConstants.SOURCE,
          redConstants.LEFT_STAGE,
          redConstants.CENTER_STAGE, redConstants.RIGHT_STAGE, redConstants.SUBWOOFER, redConstants.SHUFFLE };

    }
    return () -> new Pose3d[] { blueConstants.SPEAKER_CENTER, blueConstants.AMP, blueConstants.SOURCE,
        blueConstants.LEFT_STAGE,
        blueConstants.CENTER_STAGE, blueConstants.RIGHT_STAGE, blueConstants.SUBWOOFER, blueConstants.SHUFFLE };
  }

  /**
   * Boolean that controls when the path will be mirrored for the red
   * alliance. This will flip the path being followed to the red side of the
   * field.
   * THE ORIGIN WILL REMAIN ON THE BLUE SIDE
   * 
   * @return If we are currently on Red alliance. Will return false if no alliance
   *         is found
   */
  public static boolean isRedAlliance() {

    var alliance = FieldConstants.ALLIANCE;
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  };

  private static final class blueConstants {
    /**
     * The coordinate of the center of the blue speaker, in meters
     */
    private static final Pose3d SPEAKER_CENTER = new Pose3d(0.457 / 2, 5.557034, 2.105 - (0.133 / 2),
        new Rotation3d(0, 0, 0));
    // HEIGHT = 2.105m to the TOP of our shot. Opening is 0.133m.

    /**
     * The coordinate of the center of the blue amp, in meters.
     */
    private static final Pose3d AMP = new Pose3d(1.827, 8.2112312, (0.457 / 2) + 0.660, new Rotation3d(0, 0, 0));
    // 0.457m = The height of the AMP opening
    // 0.660m = The height between the floor and the bottom of the opening

    private static final Pose3d SOURCE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(300)));
    private static final Pose3d LEFT_STAGE = new Pose3d(
        new Pose2d(4.541771411895752, 4.736017227172852, Rotation2d.fromDegrees(120)));
    private static final Pose3d CENTER_STAGE = new Pose3d(
        new Pose2d(5.554078578948975, 4.124814033508301, Rotation2d.fromDegrees(0)));
    private static final Pose3d RIGHT_STAGE = new Pose3d(
        new Pose2d(4.524875164031982, 3.488827705383301, Rotation2d.fromDegrees(240)));

    private static final Pose3d SUBWOOFER = new Pose3d(new Pose2d(1.35, 5.50, Rotation2d.fromDegrees(180)));

    private static final Pose3d SHUFFLE = new Pose3d(
        new Pose2d(1.2991523742675781, 7.103456497192383, Rotation2d.fromDegrees(0)));
  }

  private static final class redConstants {
    /**
     * The coordinate of the center of the red speaker, in meters
     */
    private static final Pose3d SPEAKER_CENTER = new Pose3d(FIELD_LENGTH - (0.457 / 2), 5.557034, 2.105 - (0.133 / 2),
        new Rotation3d(0, 0, 0));

    /**
     * The coordinate of the center of the red speaker, in meters
     */
    private static final Pose3d FAIRBOTICS_SPEAKER = new Pose3d(16.541 - (0.457 / 2), 5.557034, 2.105 - (0.133 / 2),
        new Rotation3d(0, 0, 0));

    /**
     * The coordinate of the center of the red amp, in meters
     */
    private static final Pose3d AMP = new Pose3d(14.706, 8.2112312, (0.457 / 2) + 0.660, new Rotation3d(0, 0, 0));

    private static final Pose3d SOURCE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(60)));
    private static final Pose3d LEFT_STAGE = new Pose3d(
        new Pose2d(12.0610990524292, 3.4952545166015625, Rotation2d.fromDegrees(300)));
    private static final Pose3d CENTER_STAGE = new Pose3d(
        new Pose2d(10.983105659484863, 4.096443176269531, Rotation2d.fromDegrees(180)));
    private static final Pose3d RIGHT_STAGE = new Pose3d(
        new Pose2d(12.021082878112793, 4.7371745109558105, Rotation2d.fromDegrees(60)));

    private static final Pose3d SUBWOOFER = new Pose3d(
        new Pose2d(FIELD_LENGTH - 1.35, 5.50, Rotation2d.fromDegrees(0)));

    private static final Pose3d SHUFFLE = new Pose3d(
        new Pose2d(FIELD_LENGTH - 1.2991523742675781, 7.103456497192383, Rotation2d.fromDegrees(0)));
  }
}
