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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Driver;

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
@Autonomous(name = "JTH Autonomous Long", group = "JTH")
public class JTHAutonmousLong extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 56;
    static final double DRIVE_GEAR_REDUCTION = 20.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;

    DigitalChannel liftBottomSwitch;  // Hardware Device Object, 0 is bottom switch, Blue wire
    DigitalChannel liftTopSwitch;  // Hardware Device Object, 1 is top switch, White wire
    DigitalChannel leftBumperSwitch;
    DigitalChannel rightBumperSwitch;

    DcMotor liftMotor;
    DcMotor leftMotor;
    DcMotor rightMotor;

    Servo hookServo;
    Servo markerServo;

    double liftPower = .8;

    int step = 0;
    boolean isDocked = true;
    boolean undocking = false;
    boolean docking = false;
    boolean isHooked = false;
    boolean tankMode = true;

    double leftPOV;
    double rightPOV;
    double drivePOV;
    double turnPOV;
    double maxPOV;

//    private GoldAlignDetector detector;


    @Override
    public void runOpMode() {

        // get a reference to our digitalTouch object.
        liftBottomSwitch = hardwareMap.get(DigitalChannel.class, "liftBottomLimit"); // 0 is bottom switch, Blue wire
        liftTopSwitch = hardwareMap.get(DigitalChannel.class, "liftTopLimit"); // 1 is top switch, White wire

        leftBumperSwitch = hardwareMap.get(DigitalChannel.class, "leftBumperSwitch"); // 2 is left switch, Blue wire
        rightBumperSwitch = hardwareMap.get(DigitalChannel.class, "rightBumperSwitch"); // 3 is right switch, White wire


        // set the digital channel to input.
        liftBottomSwitch.setMode(DigitalChannel.Mode.INPUT);
        liftTopSwitch.setMode(DigitalChannel.Mode.INPUT);

        leftBumperSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightBumperSwitch.setMode(DigitalChannel.Mode.INPUT);

        // get a reference to motor objects.
        liftMotor = hardwareMap.get(DcMotor.class, "lift_drive");
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        hookServo = hardwareMap.get(Servo.class, "hook_servo");
        markerServo = hardwareMap.get(Servo.class, "marker_servo");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();

        //enableMineralDetector();

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

            encoderDrive(DRIVE_SPEED, 40, 40, 2.0);  // Forward 4 Inches with 2 Sec timeout

            markerServo.setPosition(0);

            encoderDrive(TURN_SPEED, -7, 7, 2.0);  // Backward 4 Inches with 2 Sec timeout

            encoderDrive(DRIVE_SPEED, 30, -30, 6.0);  // left turn 2 Inches with 1 Sec timeout

            encoderDrive(TURN_SPEED, -2, 2, 2.0);

            encoderDrive(DRIVE_SPEED, 80, 80, 5.0);

            encoderDrive(TURN_SPEED, -1, 1, 2.0);

            encoderDrive(DRIVE_SPEED, -100, -100, 6.0);
        }
    }




    public void unDock() {

        if ((docking == false) && (undocking == false)) {
            undocking = true;
            lowerTheRobot();
            sleep(200);
            idle();
            unHook();
            sleep(200);
            idle();
            lowerTheHook();
            isDocked = false;

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Step through each leg of the path,q
            // Note: Reverse movement is obtained by setting a negative distance (not speed)

//            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
//            telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.
//
//            if (detector.getAligned()) {
//                encoderDrive(DRIVE_SPEED, 56, 56, 4.0);  // S5: Forward 40 Inches with 4 Sec timeout
//            } else if (detector.getXPosition() < 300) {
//                encoderDrive(TURN_SPEED, 2, -2, 4.0);  // S2: Turn left 2 Inches with 4 Sec timeout
//            } else {
//                encoderDrive(TURN_SPEED, -2, 2, 4.0);  // S2: Turn Right 2 Inches with 4 Sec timeout
//            }

//            encoderDrive(DRIVE_SPEED, 6, 6, 5.0);  // S1: Forward 6 Inches with 5 Sec timeout
//            encoderDrive(TURN_SPEED, 2, -2, 4.0);  // S2: Turn Right 2 Inches with 4 Sec timeout
//            encoderDrive(DRIVE_SPEED, 6, 6, 4.0);  // S3: Forward 6 Inches with 4 Sec timeout
//            encoderDrive(TURN_SPEED, -2, 2, 4.0);  // S4: Turn 2 Inches with 4 Sec timeout
//            encoderDrive(DRIVE_SPEED, 40, 40, 4.0);  // S5: Forward 40 Inches with 4 Sec timeout

            undocking = false;
        }

    }


    public void dock() {
        if ((docking == false) && (undocking == false)) {
            docking = true;
            liftTheHook();
            sleep(200);
            idle();
            hook();
            sleep(2000);
            idle();
            liftTheRobot();
            isDocked = true;
        }
    }

    public boolean lowerTheRobot() {
        return liftTheHook();
    }

    public boolean liftTheHook() {
        while ((opModeIsActive()) && (liftTopSwitch.getState() == true)) {

            telemetry.addData("liftTopSwitch", "Is Not Pressed, Lifting the hook");
            telemetry.update();
            // Set the motor to the new power and pause;
            liftMotor.setPower(0.5);
        }
        liftMotor.setPower(0);
        return true;
    }

    public boolean liftTheRobot() {
        return lowerTheHook();
    }

    public boolean lowerTheHook() {
        while ((opModeIsActive()) && (liftBottomSwitch.getState() == true)) {
            telemetry.addData("liftBottomSwitch", liftBottomSwitch.getState());
            telemetry.update();
            // Set the motor to the new power and pause;
            liftMotor.setPower(-liftPower);
        }
        telemetry.addData("liftBottomSwitch", liftBottomSwitch.getState());
        telemetry.update();
        liftMotor.setPower(0);
        return true;
    }

    public boolean hook() {
        hookServo.setPosition(0);
        isHooked = true;
        return true;
    }

    public boolean unHook() {
        hookServo.setPosition(1);
        isHooked = false;
        return true;
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}


//    public void enableMineralDetector() {
//        // Set up detector
//        detector = new GoldAlignDetector(); // Create detector
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
//        detector.useDefaults(); // Set detector to use default settings
//
//        // Optional tuning
//        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
//        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005; //
//
//        detector.ratioScorer.weight = 5; //
//        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
//
//        detector.enable(); // Start the detector!
//    }
//
//    public void trackGoldMineral() {
//        if (detector.getAligned()) {
//            encoderDrive(TURN_SPEED, 2, 2, 4.0);  // S5: Forward 40 Inches with 4 Sec timeout
//        } else if (detector.getXPosition() < 300) {
//            encoderDrive(TURN_SPEED, 1, -1, 4.0);  // S2: Turn left 2 Inches with 4 Sec timeout
//        } else {
//            encoderDrive(TURN_SPEED, -1, 1, 4.0);  // S2: Turn Right 2 Inches with 4 Sec timeout
//        }
//    }

