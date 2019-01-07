package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Test code : To test drive and turn accuracy.
 */
@TeleOp(name = "JTH Auto Test", group = "JTH")
@Disabled
public class JTHAutoTest extends JTHOpMode {

    protected JTHAutoDepo depo;
    protected JTHAutoCrater crater;

    @Override
    public void runOpMode() {

        initRobot();
        telemetry.addLine("Mapped hardware");

        initArm();
        telemetry.addLine("Arm set to home");

        resetArmEncoders();
        telemetry.addLine("Arm encoders reset");

        enableMineralDetector();
        telemetry.addLine("Gold detector enabled");


        telemetry.addData("Gold X position", detector.getXPosition());


        telemetry.addLine("Press Start....");
        telemetry.update();

        waitForStart();
/*

        showMessageOnDriverStation("Lower the robot");
        lowerTheRobot();
        sleep(200);

        showMessageOnDriverStation("Unhook the robot");
        unHook();
        sleep(200);
*/
        // if(detector.)
        sleep(2000);

        if (detector.getXPosition() < 100) {//going left
            showMessageOnDriverStation("Gold found on the left - " + detector.getXPosition());

            showMessageOnDriverStation("Move forward");
            driveForward(3, 5);

            showMessageOnDriverStation("Turn left");
            encoderDrive(TURN_SPEED, 4, -3, 10);

            showMessageOnDriverStation("Reach out to move gold mineral");
            //reachOutToMoveGoldMineral();

            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setTargetPosition(100);
            armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

            sleep(400);

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(400);
            armMotor.setPower(armSpeed);

            sleep(400);

            wristServo.setPosition(0.7585);
            //elbowServo.setPosition(0.3766);


            elbowServo.setPosition(0.15);

            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setTargetPosition(590);
            armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

            sleep(400);

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(70);
            armMotor.setPower(armSpeed);

            sleep(400);

            //showMessageOnDriverStation("Move forward");
            driveForward(4.5, 5);

            sleep(500);


            elbowServo.setPosition(0.6);


            sleep(200);
            initArm();


            //sampling complete

            sleep(200);
            showMessageOnDriverStation("Turn back to middle");
            encoderDrive(TURN_SPEED, -4, 3, 10);


            //Goto alliance crater
            sleep(200);
            //driveForward(5, 5);

            encoderDrive(TURN_SPEED, -10, 10, 10);

            sleep(200);
            driveForward(20, 5);

            sleep(200);
            encoderDrive(TURN_SPEED, -8, 8, 10);

            reachIntoCrater();

            sleep(200);
            driveForward(18, 5);


           /* //Goto apponent crater


            sleep(200);


            encoderDrive(TURN_SPEED, 6, -6, 10);

            sleep(200);
            driveForward(24, 5);

            sleep(200);
            encoderDrive(TURN_SPEED, 7, -7, 10);

            reachIntoCrater();

            sleep(200);
            driveForward(18, 5);*/

        } else if (detector.getXPosition() > 400) {//going right
            showMessageOnDriverStation("Gold found on the right - " + detector.getXPosition());

            showMessageOnDriverStation("Move forward");
            driveForward(3, 5);

            showMessageOnDriverStation("Turn right");
            encoderDrive(TURN_SPEED, -3, 4.5, 10);

            showMessageOnDriverStation("Reach out to move gold mineral");
            //reachOutToMoveGoldMineral();

            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setTargetPosition(100);
            armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

            sleep(400);

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(400);
            armMotor.setPower(armSpeed);

            sleep(400);

            wristServo.setPosition(0.8585);
            //elbowServo.setPosition(0.3766);


            elbowServo.setPosition(0.6);

            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setTargetPosition(590);
            armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

            sleep(400);

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(100);
            armMotor.setPower(armSpeed);

            sleep(400);

            //showMessageOnDriverStation("Move forward");
            driveForward(1.5, 5);

            sleep(500);
            elbowServo.setPosition(0.15);

            sleep(200);
            initArm();

            //Sampling complete


            showMessageOnDriverStation("Turn back to middle");
            encoderDrive(TURN_SPEED, 4, -4, 10);


            //Goto alliance crater
            sleep(200);
            driveForward(5, 5);

            encoderDrive(TURN_SPEED, -10, 10, 10);

            sleep(200);
            driveForward(20, 5);

            sleep(200);
            encoderDrive(TURN_SPEED, -8, 9, 10);

            reachIntoCrater();

            sleep(200);
            driveForward(18, 5);


           /* //Goto apponent crater


            sleep(200);
            driveForward(6, 5);

            encoderDrive(TURN_SPEED, 8, -8, 10);

            sleep(200);
            driveForward(24, 5);

            sleep(200);
            encoderDrive(TURN_SPEED, 7, -7, 10);

            reachIntoCrater();

            sleep(200);
            driveForward(18, 5);
*/
        } else {
            showMessageOnDriverStation("Gold found in the middle - " + detector.getXPosition());

            showMessageOnDriverStation("Reach out to move gold mineral");
            //reachOutToMoveGoldMineral();

            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setTargetPosition(100);
            armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

            sleep(400);

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(400);
            armMotor.setPower(armSpeed);

            sleep(400);

            wristServo.setPosition(0.8585);
            //elbowServo.setPosition(0.3766);


            elbowServo.setPosition(0.6);

            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setTargetPosition(590);
            armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

            sleep(400);

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(60);
            armMotor.setPower(armSpeed);

            sleep(400);

            showMessageOnDriverStation("Move forward");
            driveForward(3.5, 5);

            sleep(500);
            elbowServo.setPosition(0.15);
//Sampling completed



            /*//Goto alliance crater

            sleep(200);
            initArm();

            sleep(200);
            driveForward(5, 5);

            encoderDrive(TURN_SPEED, -10, 10, 10);

            sleep(200);
            driveForward(20, 5);

            sleep(200);
            encoderDrive(TURN_SPEED, -8, 9, 10);

            reachIntoCrater();

            sleep(200);
            driveForward(18, 5);*/


            //Goto apponent crater


            sleep(200);
            initArm();

            sleep(200);
            driveForward(6, 5);


            sleep(2000);

            encoderDrive(TURN_SPEED, 8, -8, 10);

            sleep(200);
            driveForward(24, 5);

            sleep(200);
            encoderDrive(TURN_SPEED, 7, -7, 10);

            reachIntoCrater();

            sleep(200);
            driveForward(18, 5);


            /*sleep(200);
            initArm();
            driveForward(30, 5);
            dropMarker();
            turnRight(45);
            encoderDrive(DRIVE_SPEED, 4, 4, 5);
            turnRight(90);
            showMessageOnDriverStation("C9  :Extend Arm");
            //reachIntoCrater();
            controlArmManually = false;

            wristServo.setPosition(1);
            elbowServo.setPosition(0.437);


            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(200);
            armMotor.setPower(armSpeed);
            sleep(200);

            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setTargetPosition(100);
            armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);
            showMessageOnDriverStation("C8  :Drive Forward 30");
            driveForward(45, 5.0);          *//*  showMessageOnDriverStation("Move back");
            driveReverse(3.5, 5);

            showMessageOnDriverStation("Set arm to home");
            setArmToHome();*/

        }


        while (opModeIsActive()) {
            if (gamepad1.start) {

            } else if (gamepad1.x) {
                turnLeft(leftTurnAngle);
            } else if (gamepad1.b) {
                turnRight(rightTurnAngle);
            } else if (gamepad1.y) {
                driveForward(testDriveDistance, 5.0);
            } else if (gamepad1.a) {
                driveReverse(testDriveDistance, 5.0);
            } else if (gamepad1.left_bumper) {

            } else if (gamepad1.right_bumper) {

            } else if (gamepad1.dpad_down) {

            } else if (gamepad1.dpad_up) {

            } else if (gamepad1.dpad_right) {
                depo.path();
            } else if (gamepad1.dpad_left) {
                crater.path();
            } else {
                driveUsingPOVMode();

                if ((gamepad2.right_stick_y != 0) || (gamepad2.right_stick_x != 0)) {
                    setArmToManualControl();
                }

                if (controlArmManually) {
                    armMotor.setPower(-gamepad2.right_stick_y * armSpeed);
                    armSlideMotor.setPower(gamepad2.right_stick_x * armSlideSpeed);
                }


            }
        }
    }
}



