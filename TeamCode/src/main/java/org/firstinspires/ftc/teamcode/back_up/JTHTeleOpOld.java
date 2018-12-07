

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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


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
public class JTHTeleOpOld extends JTHOpModeOld {




    @Override
    public void runOpMode() {

        // get a reference to our digitalTouch object.
        initRobot();
        
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
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
            telemetry.addData("Button1", gamepad1.toString());
            telemetry.addData("Button2", gamepad2.toString());
            /* telemetry.addData("Gold position", detector.getXPosition());*/

            telemetry.addData("controlArmManually", controlArmManually);

            telemetry.addData("Arm Slide Position", armSlideMotor.getCurrentPosition());
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Elbow Position", elbowServo.getPosition());
            telemetry.addData("Wrist Position", wristServo.getPosition());
            telemetry.update();

            driveUsingPOVMode();
            
                        if (gamepad2.start == true) {
                setArmToHome();
            }


            if (gamepad2.dpad_down == true) {
                wristServo.setPosition(Range.clip(wristServo.getPosition() - 0.01, 0, 1));
            }

            if (gamepad2.dpad_up == true) {
                wristServo.setPosition(Range.clip(wristServo.getPosition() + 0.01, 0, 1));
            }



            if (gamepad2.x == true) {
                controlArmManually = false;

                wristServo.setPosition(0);
                elbowServo.setPosition(0.5);


                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(ARM_MAX);
                armMotor.setPower(ARM_SPEED);


                armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSlideMotor.setTargetPosition(10);
                armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);
            }

            if (gamepad2.b == true) {
                controlArmManually = false;

                wristServo.setPosition(1);
                elbowServo.setPosition(0.437);


                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(190);
                armMotor.setPower(ARM_SPEED);


                armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSlideMotor.setTargetPosition(590);
                armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);
            }

            if (gamepad2.y == true) {
                wristServo.setPosition(0.5);
                elbowServo.setPosition(Range.clip(elbowServo.getPosition() + 0.01, 0, 1));
            }

            if (gamepad2.right_bumper == true) {
                wristServo.setPosition(0.5);
            }

            if (gamepad2.left_bumper == true) {
                wristServo.setPosition(1);
            }

            if (gamepad2.dpad_right == true) {
                elbowServo.setPosition(Range.clip(elbowServo.getPosition() - 0.01, 0, 1));
            }

            if (gamepad2.dpad_left == true) {
                elbowServo.setPosition(Range.clip(elbowServo.getPosition() + 0.01, 0, 1));
            }


            if (gamepad1.left_trigger >0) {
                markerServo.setPosition(0);
            } else if (gamepad1.right_trigger >0) {
                markerServo.setPosition(1);
            } else if ((docking == false) && (undocking == false)) {
                armMotor.setPower(-gamepad2.left_stick_y * DRIVE_SPEED);
            }

            if (gamepad1.start == true) {
                unDock();
            } else if (gamepad1.x == true) {
                encoderDrive(TURN_SPEED_PRECISE, 2, -2, 1.0);  // left turn 2 Inches with 1 Sec timeout
            } else if (gamepad1.b == true) {
                encoderDrive(TURN_SPEED_PRECISE, -2, 2, 1.0);  // right turn 2 Inches with 1 Sec timeout
            } else if (gamepad1.y == true) {
                encoderDrive(DRIVE_SPEED_PRECISE, -4, -4, 2.0);  // Forward 4 Inches with 2 Sec timeout
            } else if (gamepad1.a == true) {
                encoderDrive(DRIVE_SPEED_PRECISE, 4, 4, 2.0);  // Backward 4 Inches with 2 Sec timeout
            } else if (gamepad1.left_bumper == true) {
                encoderDrive(TURN_SPEED, 2, -2, 1.0);  // left turn 2 Inches with 1 Sec timeout
            } else if (gamepad1.right_bumper == true) {
                encoderDrive(TURN_SPEED, -2, 2, 1.0);  // right turn 2 Inches with 1 Sec timeout
            } else if (gamepad1.dpad_down == true) {
                lowerTheHook();
            } else if (gamepad1.dpad_up == true) {
                liftTheHook();
            } else if (gamepad1.dpad_right == true) {
                unHook();
            } else if (gamepad1.dpad_left == true) {
                hook();
            } else if ((docking == false) && (undocking == false)) {
                driveUsingPOVMode();

                if ((gamepad2.right_stick_y != 0) || (gamepad2.right_stick_x != 0)) {
                    setArmToManualControl();
                }

                if (controlArmManually == true) {
                    armMotor.setPower(-gamepad2.right_stick_y * ARM_SPEED);
                    armSlideMotor.setPower(gamepad2.right_stick_x * ARM_SLIDE_SPEED);
                }

            }

        }

    }
}
