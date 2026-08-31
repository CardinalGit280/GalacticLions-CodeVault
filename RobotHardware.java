package org.firstinspires.ftc.team18443;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static org.firstinspires.ftc.team18443.Constants.Hardware.*;
import static org.firstinspires.ftc.team18443.Constants.Drive.*;

import androidx.annotation.*;

/**
 * <h1>Robot Hardware Abstraction Layer</h1>
 *
 * <p>
 * Provides a centralized interface for configuring and controlling the robot's
 * drivetrain, IMU, and other hardware devices. This class abstracts the
 * hardware-specific implementation from individual OpModes and provides
 * reusable methods for TeleOp and autonomous robot control.
 * </p>
 *
 * <h2>Key Capabilities</h2>
 * <ul>
 *     <li>Initializes and configures the mecanum drivetrain motors</li>
 *     <li>Initializes and configures the onboard IMU</li>
 *     <li>Provides robot-centric and field-centric mecanum driving</li>
 *     <li>Provides IMU heading, yaw reset, and heading-error utilities</li>
 *     <li>Provides normalized drivetrain power control</li>
 *     <li>Provides joystick deadzone processing</li>
 *     <li>Provides encoder-based linear and strafing movement</li>
 *     <li>Provides IMU-assisted rotational movement</li>
 * </ul>
 *
 * <h3>Example Usage</h3>
 * <pre>{@code
 * public class MyTeleOp extends LinearOpMode {
 *
 *     private RobotHardware robot;
 *
 *     @Override
 *     public void runOpMode() {
 *         robot = new RobotHardware(this);
 *         robot.init();
 *     }
 * }
 * }</pre>
 */
public class RobotHardware {

    //region Hardware Device Definitions
    // Gain access to methods in the calling OpMode
    private final LinearOpMode opMode;

    // Drive motors for the mecanum drive base
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Mechanism motors for game-specific mechanisms
    // public DcMotorEx;

    // Servos for game element manipulators
    // public Servo;

    // Inertial Measurement Unit (IMU) for orientation and field-centric control
    public IMU imu;
    //endregion

    //region Drive and Control Constants
    static final double JOYSTICK_DEADZONE = 0.1;

    public double strafeComp = 1.10; // Strafe compensation factor (empirical)
    //endregion

    //region Constructor
    // Define a constructor that allows the OpMode to pass a reference to itself
    public RobotHardware(@NonNull LinearOpMode opMode) {
        this.opMode = opMode;
    }
    //endregion

    //region Initialization
    /**
     * Initializes and configures all robot hardware.
     * <p>
     * This method maps hardware devices from the HardwareMap and applies
     * required configuration settings, including:
     * <ul>
     *   <li>Motor direction assignments</li>
     *   <li>Zero power behavior configuration</li>
     *   <li>IMU mounting orientation and initialization</li>
     * </ul>
     * <p>
     * This method must be called exactly once at the beginning of
     * {@link LinearOpMode#runOpMode() runOpMode()} before any motors,
     * sensors, or actuators are accessed.
     */
    public void init() {
        // Map motors by configuration names in the robot controller app
        frontLeft  = opMode.hardwareMap.get(DcMotorEx.class, MOTOR_FRONT_LEFT);
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, MOTOR_FRONT_RIGHT);
        backLeft   = opMode.hardwareMap.get(DcMotorEx.class, MOTOR_BACK_LEFT);
        backRight  = opMode.hardwareMap.get(DcMotorEx.class, MOTOR_BACK_RIGHT);

        // Reverse left-side drive motors so positive power moves robot forward
        // Swap these if your robot's wiring or gearboxes are mirrored
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Apply BRAKE mode to drive and mechanism motors for precise stopping
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Map servos by configuration names

        // Configure IMU mounting orientation to match physical mounting;
        // incorrect orientation will affect field-centric driving
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                // Default assumption: logo up, USB port facing forward
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        opMode.telemetry.addData(">", "Hardware Initialized");
        opMode.telemetry.update();
    }
    //endregion

    //region IMU and Heading Utilities
    /**
     * Resets the robot's heading (yaw) angle to zero.
     * <p>
     * After calling this method, the {@link IMU} will report the heading
     * relative to the robot's orientation at the time of the reset, as if
     * the robot were perfectly level at that moment.
     * <p>
     * The pitch and roll angles are unaffected, as they are always measured
     * relative to gravity and do not need resetting.
     */
    public void resetYaw() {
        if (imu != null) imu.resetYaw();
    }

    /**
     * Gets the robot's heading (yaw) in radians (-π to π) from the IMU
     *
     * @return The robot's heading in radians
     */
    @CheckResult(suggest = "heading = getHeadingRad()")
    public double getHeadingRad() {
        return AngleUnit.normalizeRadians(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );
    }

    /**
     * Gets the robot's heading (yaw) in degrees (-180° to 180°) from the IMU
     *
     * @return The robot's heading in degrees
     */
    @CheckResult(suggest = "heading = getHeadingDeg()")
    public double getHeadingDeg() {
        return AngleUnit.normalizeDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );
    }

    /**
     * Calculates the shortest angular error between the robot's current heading
     * and a target heading.
     * <p>
     * This method determines how far (and in which direction) the robot must turn
     * to reach the desired heading. The result is normalized to the range [-180, 180),
     * ensuring the smallest possible rotation is always chosen.
     *
     * @param current The robot's current heading
     * @param target  The desired target heading
     *
     * @return The normalized heading error
     */
    @CheckResult(suggest = "error = getHeadingError(current, target)")
    public double getHeadingError(double current, double target) {
        double error = target - current;
        // Normalize to [-180, 180)
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        return error;
    }
    //endregion

    //region Drive Utilities and Input Processing
    /**
     * Sets the power applied to each drivetrain motor using raw power values.
     * <p>
     * This method automatically normalizes the requested motor powers so that
     * none exceed the allowed motor power range of -1.0 to 1.0. If any input
     * magnitude is greater than 1.0, all motor powers are scaled down
     * proportionally while preserving their relative ratios. This ensures
     * predictable drivetrain behavior without motor saturation.
     *
     * @param fl The desired power for the front-left motor
     * @param fr The desired power for the front-right motor
     * @param bl The desired power for the back-left motor
     * @param br The desired power for the back-right motor
     */
    public void setDrivePower(double fl, double fr, double bl, double br) {
        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    /**
     * Sets the run mode for all drivetrain motors.
     * <p>
     * This method applies the specified {@link DcMotor.RunMode} to all drive
     * motors simultaneously, ensuring consistent motor behavior across the
     * drivetrain.
     *
     * @param mode The {@link DcMotor.RunMode} to apply to all drivetrain motors
     */
    public void setRunMode(@NonNull DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    /**
     * Applies a deadzone to joystick input to eliminate small unintended
     * movements.
     * <p>
     * Many gamepad joysticks do not return perfectly to zero, which can cause
     * unwanted robot motion. This method treats small input values near zero
     * as zero to prevent drift.
     * <p>
     * Inputs outside the deadzone are scaled so that full joystick travel still
     * produces full output, preserving smooth and predictable driver control.
     *
     * @param input Raw joystick input value from the gamepad
     *
     * @return The joystick value after applying the deadzone and rescaling
     */
    @CheckResult(suggest = "input = applyJoystickDeadzone(input)")
    public double applyJoystickDeadzone(double input) {
        if (Math.abs(input) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        return (input - Math.signum(input) * JOYSTICK_DEADZONE) /
                (1.0 - JOYSTICK_DEADZONE);
    }
    //endregion

    //region Drivetrain Control Methods
    /**
     * Drives the robot in a robot-centric manner using a mecanum drive.
     * <p>
     * In robot-centric control, the robot's motion is relative to its
     * own orientation, rather than the field. This method directly maps
     * the inputs to the robot's movement without adjustments based on
     * its heading.
     * <p>
     * Pushing the joystick forward will cause the robot to move forward
     * relative to the direction that it is currently facing.
     *
     * @param x  Lateral movement (strafe left/right)
     * @param y  Longitudinal movement (forward/backward)
     * @param rx Rotational movement (clockwise/counterclockwise)
     *
     * @see #driveFieldCentric(double, double, double)
     */
    public void driveRobotCentric(double x, double y, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;
        setDrivePower(fl, fr, bl, br);
    }

    /**
     * Drives the robot in a field-centric manner using a mecanum drive.
     * <p>
     * In field-centric control, the robot's motion is relative to the field,
     * rather than its own orientation. This method uses the {@link IMU}
     * heading to adjust the inputs so they are corrected for the robot's
     * orientation at the time of control.
     * <p>
     * Pushing the joystick forward will cause the robot to move forward
     * relative to the field, regardless of which direction it is currently
     * facing.
     *
     * @param x  Lateral movement (strafe left/right)
     * @param y  Longitudinal movement (forward/backward)
     * @param rx Rotational movement (clockwise/counterclockwise)
     *
     * @see #driveRobotCentric(double, double, double)
     */
    public void driveFieldCentric(double x, double y, double rx) {
        x *= strafeComp;
        double heading = getHeadingRad();
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        driveRobotCentric(rotX, rotY, rx);
    }
    //endregion

    //region Autonomous Drivetrain Movement
    /**
     * Moves the robot forward or backward a specified distance in inches at a
     * given speed.
     */
    @WorkerThread
    public void moveToPosition(double inches, double speed) {
        // Determine new target position and pass to motor controller
        int moveCounts = (int)(Math.round(inches * COUNTS_PER_INCH));
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + moveCounts);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + moveCounts);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + moveCounts);
        backRight.setTargetPosition(backRight.getCurrentPosition() + moveCounts);

        // Turn On RUN_TO_POSITION
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = Math.abs(speed);
        setDrivePower(power, power, power, power);

        // Loop until all motors have reached their targets
        while (opMode.opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy()
                && backLeft.isBusy() && backRight.isBusy())) {
            opMode.telemetry.addData("Drive", "Moving...");
            opMode.telemetry.update();

            opMode.idle();
        }

        // Stop all motion
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * Rotates the robot by a specified angle using IMU-based yaw measurements
     * <p>
     * This method performs a two-phase rotation for improved accuracy:
     * <ul>
     *   <li>Phase 1: Coarse rotation to reach within ~10° of the target angle</li>
     *   <li>Phase 2: Fine rotation for higher precision (±5° tolerance)</li>
     * </ul>
     *
     * @param degrees -
     * @param speedDirection -
     */
    @WorkerThread
    public void turnWithGyro(double degrees, double speedDirection) {
        // Create an object to receive the IMU angles
        double heading = getHeadingDeg();

        // Determine target heading
        double targetHeading = AngleUnit.normalizeDegrees(heading + degrees);

        // Phase 1: Coarse turn
        while (opMode.opModeIsActive()) {
            heading = getHeadingDeg();
            double error = getHeadingError(heading, targetHeading);

            if (Math.abs(error) < 10) break;

            turnWithEncoder(speedDirection);
        }

        // Phase 2: Fine adjustment
        while (opMode.opModeIsActive()) {
            heading = getHeadingDeg();
            double error = getHeadingError(heading, targetHeading);

            if (Math.abs(error) < 2) break;

            turnWithEncoder(speedDirection / 3);
        }

        // Stop all motion
        setDrivePower(0, 0, 0, 0);
    }
    public void turnWithEncoder(double input) {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontLeft.setPower(input);
        frontRight.setPower(-input);
        backLeft.setPower(input);
        backRight.setPower(-input);
    }

    /**
     * Strafes robot left or right using encoder targets.
     * Negative input for inches results in left strafing
     */
    @WorkerThread
    public void strafeToPosition(double inches, double speed) {
        // Determine new target position and pass to motor controller
        int moveCounts = (int)(Math.round(inches * COUNTS_PER_INCH * strafeComp));
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + moveCounts);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - moveCounts);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - moveCounts);
        backRight.setTargetPosition(backRight.getCurrentPosition() + moveCounts);

        // Turn On RUN_TO_POSITION
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = Math.abs(speed);
        setDrivePower(power, power, power, power);

        // Loop until all motors have reached their targets
        while (opMode.opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy()
                && backLeft.isBusy() && backRight.isBusy())) {
            opMode.telemetry.addData("Drive", "Strafing...");
            opMode.telemetry.update();

            opMode.idle();
        }

        // Stop all motion
        setDrivePower(0, 0, 0, 0);
    }
    //endregion
}
