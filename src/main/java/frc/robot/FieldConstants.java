package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
   * @return An array of field element positions. 0 = Your Speaker, 1 = Your Amp,
   *         2 = Opposing Speaker, 3 = Opposing Amp
   */
  public static Pose2d[] GET_FIELD_POSITIONS() {
    if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
      return new Pose2d[] { redConstants.SPEAKER, redConstants.AMP, blueConstants.SPEAKER,
          blueConstants.AMP };
    }
    return new Pose2d[] { blueConstants.SPEAKER, blueConstants.AMP, redConstants.SPEAKER,
        redConstants.AMP };
  }

  private static final class blueConstants {
    // TODO: add trap positions?
    /**
     * The coordinate of the center of the blue speaker, in meters
     */
    private static final Pose2d SPEAKER = new Pose2d(0, 5.547, new Rotation2d(0));

    /**
     * The coordinate of the center of the blue amp, in meters
     */
    private static final Pose2d AMP = new Pose2d(1.827, 8.2112312, new Rotation2d(0));
  }

  private static final class redConstants {
    /**
     * The coordinate of the center of the red speaker, in meters
     */
    private static final Pose2d SPEAKER = new Pose2d(16.5410515, 5.547, new Rotation2d(0));

    /**
     * The coordinate of the center of the red amp, in meters
     */
    private static final Pose2d AMP = new Pose2d(14.706, 8.2112312, new Rotation2d(0));
  }
}
