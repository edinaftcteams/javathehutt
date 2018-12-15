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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

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
@Autonomous(name = "JTH Autonomous Short", group = "JTH")
@Disabled
public class JTHAutonomous extends JTHOpModeOld {


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

            unDock();
//high noon
            openArm();

            encoderDrive(DRIVE_SPEED, 45, 45, 5.0);  // Forward 4 Inches with 2 Sec timeout

            sleep(500);

            markerServo.setPosition(0);

            encoderDrive(TURN_SPEED, 9, -9, 2.0);  // Backward 4 Inches with 2 Sec timeout

            encoderDrive(DRIVE_SPEED, -90, -90, 6.0);  // left turn 2 Inches with 1 Sec timeout

            wristServo.setPosition(0);
            elbowServo.setPosition(0.5);


            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(ARM_MAX);
            armMotor.setPower(ARM_SPEED);


            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setTargetPosition(10);
            armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);
            break;
        }
    }
}



