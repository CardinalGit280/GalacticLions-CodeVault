package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="StraferTeleOp", group="TeleOp")
// @Disabled
public class StraferTeleOp_Iterative extends OpMode {

    RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.resetYaw();

        // Send telemetry message to signify robot waiting
        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Read joystick inputs
        double forward = -gamepad1.left_stick_y; // Forward/backward
        double strafe  = gamepad1.left_stick_x;  // Strafe left/right
        double rotate = gamepad1.right_stick_x; // Rotation
        
        // Drive the robot using field-centric control
        robot.driveFieldCentric(forward, strafe, rotate);
        
        // Reset the IMU heading to zero manually by pressing the 'guide' button
        // Useful for correcting drift or re-aligning the robot to the field orientation
        if (gamepad1.guide) {
            robot.resetHeading();
        }
    }
    
    @Override
    public void stop() {
        
    }
}
