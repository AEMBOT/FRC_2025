package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class PositionConstants {
      // Defines new variables for the x/y translations for the target positions (currently at
      // placeholders)
      // Origin to bumper ~0.4572 m
      public static final double targetLevel1X = 0.3048 + 0.4572; // TODO: Measure
      public static final double targetLevel2X = 0.3048 + 0.4572; // TODO: Measure
      public static final double targetLevel3X = 0.3048 + 0.4572; // TODO: Measure
      public static final double targetLevel4X = 0.4064 + 0.4572; // Measured but not exact

      public static final double targetY = 0.1793875;

      public static final double targetAngle = Radians.convertFrom(180, Degrees);

      // Define reef centerpoints (blue alliance)
      public static final double reefCenterX = 4.489323;
      public static final double reefCenterY = 4.0259;
}
