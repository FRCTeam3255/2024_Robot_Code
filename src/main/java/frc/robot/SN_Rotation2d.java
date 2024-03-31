package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class SN_Rotation2d extends Rotation2d {

  public Measure<Angle> getMeasure() {
    return Units.Radians.of(getRadians());
  }
}
