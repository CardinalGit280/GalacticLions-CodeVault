package org.firstinspires.ftc.team18443;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.bylazar.configurables.annotations.Configurable;
import androidx.annotation.*;

/**
 * <h1>Robot Constants</h1>
 *
 * <p>Centralized constants for hardware names and robot configuration.</p>
 */
@Configurable
public class Constants {

  // Private constructor to prevent unnecessary instantiation
  private Constants() {}
  
  /** Hardware device names */
  public static class Hardware {
    @NonNull public static String MOTOR_FRONT_LEFT  = "fl";
    @NonNull public static String MOTOR_FRONT_RIGHT = "fr";
    @NonNull public static String MOTOR_BACK_LEFT   = "bl";
    @NonNull public static String MOTOR_BACK_RIGHT  = "br";

    public static RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIR = 
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIR = 
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
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
}
