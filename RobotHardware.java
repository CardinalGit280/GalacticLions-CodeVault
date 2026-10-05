package org.firstinspires.ftc.team5898;

import androidx.annotation.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.team18443.Constants.Hardware;

import static org.firstinspires.ftc.team5898.Constants.*;

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

    // Internal references to the calling OpMode
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Drive motors for the mecanum drive base
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Mechanism motors for game-specific mechanisms
    // public DcMotorEx;

    // Servos for game element manipulators
    // public Servo;

    // Inertial Measurement Unit (IMU) for orientation
    public IMU imu;

    //endregion

    //region Drive and Control Constants

    static final double JOYSTICK_DEADZONE = 0.1;
    public double strafeComp = 1.10;

    //endregion

    //region Constructor

    /**
     * Creates a constructor that stores a reference of the specified OpMode's 
     * {@link HardwareMap} and {@link Telemetry}. Hardware devices are not mapped 
     * or initialized until {@link #init()} is called.
     *
     * @param opMode the OpMode that owns and gives access to the robot hardware
     */
    public RobotHardware(@NonNull OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    //endregion

    //region Initialization

    /**
     * Retrieves the defined actuators and IMU from the {@link HardwareMap}, 
     * configures actuator directions and zero-power behavior, and initializes 
     * the IMU using the REV Hub's physical mounting orientation.
     * 
     * The method should be called once during the OpMode's initialization phase,
     * before attempting to control the drivetrain or read the IMU.
     */
    public void init() {
        // Map drivetrain motors using the names configured in the Robot Controller app.
        frontLeft  = hardwareMap.get(DcMotorEx.class, Hardware.MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, Hardware.MOTOR_FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotorEx.class, Hardware.MOTOR_BACK_LEFT);
        backRight  = hardwareMap.get(DcMotorEx.class, Hardware.MOTOR_BACK_RIGHT);

        // Reverse the left-side motors so positive power corresponds to forward motion.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Brake the drivetrain when power is set to zero.
        for (DcMotorEx motor : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Map and initialize the IMU.
        //
        // These directions must match the physical orientation of the REV Hub.
        // An incorrect orientation will produce incorrect heading measurements
        // and can cause field-centric driving to behave incorrectly.
        imu = hardwareMap.get(IMU.class, Hardware.IMU);

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();
    }

    //endregion

    //region IMU and Heading Utilities

    /**
     * Resets the IMU's yaw angle to zero so the following yaw measurements are 
     * relative to the robot's orientation. This is useful for establishing a 
     * new heading reference before field-centric driving or autonomous rotation.
     * 
     * Note that resetting yaw does not physically rotate the robot and does not 
     * reset pitch or roll.
     */
    public void resetYaw() {
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

    /**
     * Calculates the shortest signed angular error between two headings.
     *
     * @param current the current heading in degrees
     * @param target the desired heading in degrees
     *
     * @return shortest signed angular error from current to target,
     *         in degrees
     */
    @CheckResult(suggest = "error = getHeadingError(current, target)")
    public double getHeadingError(double current, double target) {
        double error = target - current;

        // Normalize the error to the shortest path: [-180°, 180°).
        while (error > 180) {
            error -= 360;
        }

        while (error <= -180) {
            error += 360;
        }

        return error;
    }

    //endregion

    //region Drive Utilities and Input Processing

    /**
     * Sets drivetrain motor power while automatically normalizing the inputs.
     *
     * @param fl the desired front-left motor power
     * @param fr the desired front-right motor power
     * @param bl the desired back-left motor power
     * @param br the desired back-right motor power
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
     * Stops all drivetrain motors by setting their power to zero while 
     * retaining their configured zero-power behavior.
     */
    public void stopDriveMotors() {
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * Sets the same {@link DcMotor.RunMode} on all four drivetrain motors.
     *
     * @param mode the RunMode to apply to every drivetrain motor
     */
    public void setRunMode(@NonNull DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

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
     * @param x  the lateral translation input
     * @param y  the longitudinal translation input
     * @param rx the rotational input
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
     * Drives the robot using field-centric mecanum control. In this mode,
     * the translation inputs are interpreted relative to the field rather than
     * the robot. 
     *
     * The IMU heading is used to rotate the requested translation vector into 
     * the robot's coordinate system before applying any drive calculations.
     *
     * @param x  the lateral field-relative translation input
     * @param y  the longitudinal field-relative translation input
     * @param rx the rotational input
     */
    public void driveFieldCentric(double x, double y, double rx) {
        double heading = getHeadingRad();

        // Rotate the field-relative translation vector into robot coordinates.
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        driveRobotCentric(rotX, rotY, rx);
    }

    //endregion
    
    //region Autonomous Drivetrain Movement

    /**
     * Returns the owning OpMode as a {@link LinearOpMode}, if applicable.
     *
     * @return the owning OpMode as a {@link LinearOpMode}, or null if
     *         the owning OpMode is not a LinearOpMode
     */
    private LinearOpMode getLinearOpMode() {
        if (!(opMode instanceof LinearOpMode)) {
            return null;
        }

        return (LinearOpMode) opMode;
    }

    /**
     * Moves the robot forward or backward by a specified distance using
     * drivetrain encoders.
     *
     * @param inches distance to travel in inches; positive values move forward,
     *               negative values move backward
     * @param speed motor power magnitude, normally in the range [0, 1]
     */
    @WorkerThread
    public void moveToPosition(double inches, double speed) {
        LinearOpMode linearOpMode = getLinearOpMode();

        if (linearOpMode == null) {
            stopDrive();
            return;
        }

        // Convert the requested distance to encoder counts.
        int moveCounts = (int) Math.round(inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + moveCounts);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + moveCounts);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + moveCounts);
        backRight.setTargetPosition(backRight.getCurrentPosition() + moveCounts);

        // Command all drivetrain motors to move toward their targets.
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = Math.abs(speed);
        setDrivePower(power, power, power, power);

        // Continue while the OpMode is active and all motors are still moving.
        while (linearOpMode.opModeIsActive()
                && frontLeft.isBusy()
                && frontRight.isBusy()
                && backLeft.isBusy()
                && backRight.isBusy()) {

            telemetry.addData("Drive", "Moving...");
            telemetry.update();

            linearOpMode.idle();
        }

        stopDrive();
    }

    /**
     * Rotates the robot by the specified number of degrees using the IMU for
     * heading feedback.
     *
     * @param degrees relative rotation in degrees; positive and negative values
     *                represent opposite rotation directions
     * @param speedDirection signed motor power used for the coarse rotation;
     *                       its magnitude should normally be in [0, 1]
     */
    @WorkerThread
    public void turnWithGyro(double degrees, double speedDirection) {
        LinearOpMode linearOpMode = getLinearOpMode();

        if (linearOpMode == null) {
            stopDrive();
            return;
        }

        double heading = getHeadingDeg();

        // Calculate the target heading and normalize it to [-180°, 180°).
        double targetHeading = AngleUnit.normalizeDegrees(heading + degrees);

        // Coarse rotation.
        while (linearOpMode.opModeIsActive()) {
            heading = getHeadingDeg();
            double error = getHeadingError(heading, targetHeading);

            if (Math.abs(error) < 10) {
                break;
            }

            turnWithEncoder(speedDirection);
        }

        // Fine rotation at one-third of the coarse-turn power.
        while (linearOpMode.opModeIsActive()) {
            heading = getHeadingDeg();
            double error = getHeadingError(heading, targetHeading);

            if (Math.abs(error) < 2) {
                break;
            }

            turnWithEncoder(speedDirection / 3);
        }

        stopDrive();
    }

    /**
     * Applies open-loop rotational power to the drivetrain.
     *
     * @param input signed rotational motor power; the sign determines the
     *             direction of rotation and the magnitude determines speed
     */
    public void turnWithEncoder(double input) {
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(input);
        frontRight.setPower(-input);
        backLeft.setPower(input);
        backRight.setPower(-input);
    }

    /**
     * Strafes the robot by a specified distance using drivetrain encoders.
     *
     * @param inches distance to strafe in inches
     * @param speed motor power magnitude, normally in the range [0, 1]
     */
    @WorkerThread
    public void strafeToPosition(double inches, double speed) {
        LinearOpMode linearOpMode = getLinearOpMode();

        if (linearOpMode == null) {
            stopDrive();
            return;
        }

        // Convert the requested lateral distance to encoder counts.
        double moveCounts = inches * COUNTS_PER_INCH * strafeComp;
        int counts = (int) Math.round(moveCounts);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + counts);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - counts);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - counts);
        backRight.setTargetPosition(backRight.getCurrentPosition() + counts);

        // Command all drivetrain motors to move toward their targets.
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = Math.abs(speed);
        setDrivePower(power, power, power, power);

        // Continue while the OpMode is active and all motors are still moving.
        while (linearOpMode.opModeIsActive()
                && frontLeft.isBusy()
                && frontRight.isBusy()
                && backLeft.isBusy()
                && backRight.isBusy()) {

            telemetry.addData("Drive", "Strafing...");
            telemetry.update();

            linearOpMode.idle();
        }

        stopDrive();
    }

    //endregion
}
