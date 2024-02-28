package frc.robot;

import java.util.Optional;

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
   * Gets the positions of all of the necessary field elements on the field. All
   * coordinates are in meters and are relative to the blue alliance. See
   * <a href=
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
   * Robot Coordinate Systems</a>
   * 
   * @return An array of field element positions. Your Speaker, Amp, Source, Left
   *         Stage, Center Stage, Right Stage. Then, it continues in the same
   *         order
   *         with the positions on the opposing alliance
   */
  public static Pose3d[] GET_FIELD_POSITIONS() {
    if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
      return new Pose3d[] { redConstants.SPEAKER, redConstants.AMP, redConstants.SOURCE, redConstants.LEFT_STAGE,
          redConstants.CENTER_STAGE, redConstants.RIGHT_STAGE, blueConstants.SPEAKER, blueConstants.AMP,
          blueConstants.SOURCE, blueConstants.LEFT_STAGE, blueConstants.CENTER_STAGE, blueConstants.RIGHT_STAGE };

    }
    return new Pose3d[] { blueConstants.SPEAKER, blueConstants.AMP, blueConstants.SOURCE, blueConstants.LEFT_STAGE,
        blueConstants.CENTER_STAGE, blueConstants.RIGHT_STAGE, redConstants.SPEAKER, redConstants.AMP,
        redConstants.SOURCE, redConstants.LEFT_STAGE, redConstants.CENTER_STAGE, redConstants.RIGHT_STAGE };
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
    private static final Pose3d SPEAKER = new Pose3d(0, 5.619077205657959, 1.552, new Rotation3d(0, 0, 0));

    /**
     * The coordinate of the center of the blue amp, in meters.
     */
    private static final Pose3d AMP = new Pose3d(1.827, 8.2112312, (0.457 / 2) + 0.660, new Rotation3d(0, 0, 0));
    // 0.457m = The height of the AMP opening
    // 0.660m = The height between the floor and the bottom of the opening

    // TODO: Update Translational Positions (AFTER PHR)
    private static final Pose3d SOURCE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(-60)));
    private static final Pose3d LEFT_STAGE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(120)));
    private static final Pose3d CENTER_STAGE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    private static final Pose3d RIGHT_STAGE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(-120)));
  }

  private static final class redConstants {
    /**
     * The coordinate of the center of the red speaker, in meters
     */
    private static final Pose3d SPEAKER = new Pose3d(16.5410515, 5.619077205657959, 1.552, new Rotation3d(0, 0, 0));

    /**
     * The coordinate of the center of the red amp, in meters
     */
    private static final Pose3d AMP = new Pose3d(14.706, 8.2112312, (0.457 / 2) + 0.660, new Rotation3d(0, 0, 0));

    private static final Pose3d SOURCE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(-119)));
    private static final Pose3d LEFT_STAGE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(-60)));
    private static final Pose3d CENTER_STAGE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
    private static final Pose3d RIGHT_STAGE = new Pose3d(new Pose2d(0, 0, Rotation2d.fromDegrees(60)));

  }
}
