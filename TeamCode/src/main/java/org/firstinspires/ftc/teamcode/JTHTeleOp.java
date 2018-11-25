

/* Copyright (c) 2017 FIRST. All rights reserved. */

/*
 * Sample code for JavaTheHUTT - Raj Kammela 11/11/2018
 * 1. Two drive modes. Start button toggles between the modes. Default is Tank mode. You can see if robot is in Tank mode or not in driver station telemetry log
 *    a. Tank mode : control left motor with left stick, right motor with right stick
 *    b. POV mode  : Left stick controls forward/back word movement, like accelerator in the car. Right sticl controls turns, like steering wheel.
 * 2. Directional pad controls lift up/down and hook left/right
 * 3. Button A: Initiates autonomous docking
 * 4. Button B: Initiates autonomous undocking
 * 5. Button X: Move robot forward 4 inches
 * 5. Button Y: Move robot reverse 4 inches
 * 6. Left Bumper: Turn left 2 inches
 * 7. Right Bumper: Turn right 2 inches
 * 8. Guide : It starts gold mineral tracking. Robot should automatically align itself to gold mineral and move towards it.
 */

package org.firstinspires.ftc.teamcode;

//
//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Sample code for JavaTheHUTT - Raj Kammela 11/11/2018
 * 1. Two drive modes. Start button toggles between the modes. Default is Tank mode. You can see if robot is in Tank mode or not in driver station telemetry log
 *    a. Tank mode : control left motor with left stick, right motor with right stick
 *    b. POV mode  : Left stick controls forward/back word movement, like accelerator in the car. Right stick controls turns, like steering wheel.
 * 2. Directional pad controls lift up/down and hook left/right
 * 3. Button A: Initiates autonomous docking
 * 4. Button B: Initiates autonomous undocking
 * 5. Button X: Move robot forward 4 inches
 * 5. Button Y: Move robot reverse 4 inches
 * 6. Left Bumper: Turn left 2 inches
 * 7. Right Bumper: Turn right 2 inches
 */
@TeleOp(name = "JTH Teleop", group = "JTH")
public class JTHTeleOp extends JTHOpMode {




    @Override
    public void runOpMode() {

        // get a reference to our digitalTouch object.
        initRobot();
        waitForStart();

        // while the op mode is active, use game pad to drive in tank mode, or initiate dock/un dock procedures
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            telemetry.addLine();
            telemetry.addData("Button", gamepad1.toString());
            telemetry.addData("Tank Mode", tankMode);
            telemetry.addData("isDocked", isDocked);
            telemetry.addData("isHooked", isHooked);
            telemetry.addData("Step", step);
            telemetry.update();

            driveUsingPOVMode();

            if (gamepad1.left_trigger >0) {
                markerServo.setPosition(0);
            } else if (gamepad1.right_trigger >0) {
                markerServo.setPosition(1);
            } else if ((docking == false) && (undocking == false)) {
                armMotor.setPower(-gamepad2.left_stick_y * DRIVE_SPEED);
            }

            if (gamepad1.a == true) {
                dock();
//            } else if (gamepad1.b == true) {
//                unDock();
            } else if (gamepad1.x == true) {
                encoderDrive(DRIVE_SPEED, 4, 4, 2.0);  // Backward
            }
            else if (gamepad1.y == true) {
                encoderDrive(DRIVE_SPEED, -4, -4, 2.0);  // Forward 4 Inches with 2 Sec timeout
            } else if (gamepad1.left_bumper == true) {
                encoderDrive(TURN_SPEED, 6, -6, 1.0);  // left turn 2 Inches with 1 Sec timeout
            } else if (gamepad1.right_bumper == true) {
                encoderDrive(TURN_SPEED, 12, -12, 1.0);  // right turn 2 Inches with 1 Sec timeout
            } else if (gamepad1.dpad_down == true) {
                lowerTheHook();
            } else if (gamepad1.dpad_up == true) {
                liftTheHook();
            } else if (gamepad1.dpad_right == true) {
                unHook();
            }
              else if (gamepad1.left_bumper){
                //drop marker
            } else if (gamepad1.dpad_left == true) {
                hook();
            } else if (gamepad1.start == true) {
                tankMode = !tankMode;
            } else if (gamepad1.guide == true) {
                //trackGoldMineral();
            } else if (gamepad1.back == true) {
                docking = false;
                undocking = false;
            } else if ((docking == false) && (undocking == false)) {
                if (tankMode == true) {
                    driveUsingTankMode();
                } else {
                    driveUsingPOVMode();
                }
            }

        }

    }
}
