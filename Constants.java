package org.firstinspires.ftc.team18443;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import androidx.annotation.*;

/**
 * Robot Constants
 *
 * Centralized constants for hardware names and robot configuration.
 */
public class Constants {

  // Private constructor to prevent unnecessary set up
  private Constants() {}
  
  /** Hardware device names */
  public static class Hardware {
    // Drivetrian motors
    @NonNull public static String MOTOR_FRONT_LEFT  = "fl";
    @NonNull public static String MOTOR_FRONT_RIGHT = "fr";
    @NonNull public static String MOTOR_BACK_LEFT   = "bl";
    @NonNull public static String MOTOR_BACK_RIGHT  = "br";

    // Mechanism motors
    // @NonNull public static String MOTOR_X = "x";

    // Servos
    // @NonNull public static String SERVO_X = "x";

    // Inertial Measurement Unit
    @NonNull public static String IMU = "imu";
  }
  
  /** Robot physical dimensions and drivetrain parameters */
  public static class Drive {
    public static double COUNTS_PER_ROTATION   = 537.7;
    public static double WHEEL_DIAMETER_INCHES = 3.779;
    public static double DRIVE_GEAR_REDUCTION  = 1.0;
    public static double COUNTS_PER_INCH =
        (COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * Math.PI);
  }

  /* Filtered PID contoller values */
  @Configurable
  public static class PIDF {
    @Sorter(sort = 0)
    public static double kP = 0.0;
    @Sorter(sort = 1)
    public static double kI = 0.0;
    @Sorter(sort = 2)
    public static double kD = 0.0;
    @Sorter(sort = 3)
    public static double kF = 0.0;
  }
}
