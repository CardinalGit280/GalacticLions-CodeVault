package org.firstinspires.ftc.team18443;

import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;

/**
 * <h1>Robot Constants</h1>
 *
 * <p>
 * </p>
 *
 * <h2>Key Capabilities</h2>
 * <ul>
 *     <li></li>
 * </ul>
 */
@Configurable
public class Constants {

  // Private constructor to prevent unnecessary instantiation
  private Constants() {}

  // Hardware device name strings
  public static String MOTOR_FRONT_LEFT  = "fl";
  public static String MOTOR_FRONT_RIGHT = "fr";
  public static String MOTOR_BACK_LEFT   = "bl";
  public static String MOTOR_BACK_RIGHT  = "br";

  // Robot physical & tuning parameters
  public static double COUNTS_PER_ROTATION   = 537.7;
  public static double WHEEL_DIAMETER_INCHES = 3.779;
  public static double DRIVE_GEAR_REDUCTION  = 1.0;
  static final double  COUNTS_PER_INCH       = (COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION) 
                                               / (WHEEL_DIAMETER_INCHES * Math.PI);
}
