package frc.robot.commands.autos;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface AutoInterface {
  public Supplier<Pose2d> getInitialPose();

  public Command getAutonomousCommand();
}
