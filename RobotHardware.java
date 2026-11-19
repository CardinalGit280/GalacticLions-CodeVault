package org.firstinspires.ftc.team5898;

import androidx.annotation.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.team5898.Constants.*;

/**
 * Provides a main dashboard for initializing and controlling the robot's
 * hardware. The class abstracts FTC SDK hardware from individual OpModes 
 * and provides reusable drivetrain, IMU, and autonomous movement utilities.
 *
 * The class is intended to be set up with an {@link OpMode} and initialized
 * by calling {@link #init()} before any hardware-dependent methods are used.
 */
public class RobotHardware {

    //region Hardware Device Definitions

    // Drive motors for the mecanum drive base
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final DcMotorEx[] driveMotors;

    // Mechanism motors for game-specific mechanisms
    // private final DcMotorEx;

    // Servos for game element manipulators
    // private final Servo;

    // Inertial Measurement Unit (IMU) for orientation
    private final IMU imu;

    //endregion

    //region Drive and Control Constants

    static final double JOYSTICK_DEADZONE = 0.1;
    public double strafeComp = 1.10;

    //endregion

    //region Initialization

    /**
     * Retrieves the defined actuators and IMU from the {@link HardwareMap}, 
     * configures actuator directions and zero-power behavior, and initializes 
     * the IMU using the REV Hub's physical mounting orientation.
     * 
     * The method should be called once during the OpMode's initialization phase,
     * before attempting to control the drivetrain or read the IMU.
     *
     * @param hardwareMap The hardware map from the calling OpMode
     */
    public void init(@NonNull HardwareMap hardwareMap) {
        // Map motors using the names configured in Constants.java.
        frontLeft  = hardwareMap.get(DcMotorEx.class, Hardware.MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, Hardware.MOTOR_FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotorEx.class, Hardware.MOTOR_BACK_LEFT);
        backRight  = hardwareMap.get(DcMotorEx.class, Hardware.MOTOR_BACK_RIGHT);

        driveMotors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        // Reverse the left-side motors so positive power corresponds to forward motion.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        
        // Brake the drivetrain when power is set to zero.
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Map and initialize the IMU.
        //
        // These directions must match the physical orientation of the REV Hub.
        // An incorrect orientation will produce incorrect heading measurements
        // and can cause field-centric driving to behave incorrectly.
        imu = hardwareMap.get(IMU.class, Hardware.IMU);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();
    }

    //endregion

    //region IMU and Heading Utilities

    /**
     * Resets the IMU's heading angle to zero so the following measurements are 
     * relative to the robot's orientation. This is useful for establishing a 
     * new reference before field-centric driving or autonomous rotation.
     * 
     * Note that resetting does not physically rotate the robot and does not 
     * reset pitch or roll.
     */
    public void resetHeading() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    /**
     * Returns the robot's current IMU yaw angle in radians which is normalized 
     * to the range [-π, π).
     *
     * @return the current robot heading in radians
     */
    @CheckResult(suggest = "heading = getHeadingRad()")
    public double getHeadingRad() {
        return AngleUnit.normalizeRadians(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );
    }

    /**
     * Returns the robot's current IMU yaw angle in degrees which is normalized 
     * to the range [-180°, 180°).
     *
     * @return the current robot heading in degrees
     */
    @CheckResult(suggest = "heading = getHeadingDeg()")
    public double getHeadingDeg() {
        return AngleUnit.normalizeDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );
    }
    
    //endregion

    //region Drive Utilities and Input Processing

    /**
     * Applies the configured joystick deadzone and rescales the remaining input.
     *
     * @param input the raw joystick input, in the range [-1, 1]
     *
     * @return deadzone-adjusted joystick input
     */
    @CheckResult(suggest = "input = applyJoystickDeadzone(input)")
    public double applyJoystickDeadzone(double input) {
        if (Math.abs(input) < JOYSTICK_DEADZONE) {
            return 0.0;
        }

        return (input - Math.signum(input) * JOYSTICK_DEADZONE)
                / (1.0 - JOYSTICK_DEADZONE);
    }

    //endregion

    //region Drivetrain Control Methods

    /**
     * Drives the robot using robot-centric mecanum control. In this mode,
     * the translation inputs are interpreted relative to the robot itself.
     *
     * For example, a positive forward input always results in motion toward 
     * the robot's front, independent of its orientation on the field.
     *
     * @param forward    The lateral translation input
     * @param strafe     The longitudinal translation input
     * @param turn       The rotational input
     *
     * @see #driveFieldCentric(double, double, double)
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {      
        double denominator = Math.max(1.0,
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
        frontLeft.setPower((forward + strafe + turn) / denominator);
        backLeft.setPower((forward - strafe + turn) / denominator);
        frontRight.setPower((forward - strafe - turn) / denominator);
        backRight.setPower((forward + strafe - turn) / denominator);
    }

    /**
     * Drives the robot using field-centric mecanum control. In this mode,
     * the translation inputs are interpreted relative to the field rather than
     * the robot. 
     *
     * The IMU heading is used to rotate the requested translation vector into 
     * the robot's coordinate system before applying any drive calculations.
     *
     * @param forward    The lateral translation input
     * @param strafe     The longitudinal translation input
     * @param turn       The rotational input
     *
     * @see #driveRobotCentric(double, double, double)
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeadingRad();

        // Rotate the field-relative translation vector into robot coordinates.
        double rotatedX = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double rotatedY = strafe * Math.cos(heading) + forward * Math.sin(heading);

        driveRobotCentric(rotatedX, rotatedY, turn);
    }

    //endregion
    
    //region Autonomous Drivetrain Movement

    //endregion
}
