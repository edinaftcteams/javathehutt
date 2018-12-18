package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Test code : To test drive and turn accuracy.
 */
@TeleOp(name = "JTH Auto Test", group = "JTH")
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

            //showMessageOnDriverStation("Move back");
            //driveReverse(8, 5);

            //showMessageOnDriverStation("Set arm to home");
            //setArmToHome();

            //showMessageOnDriverStation("Align robot to lander");
            //encoderDrive(TURN_SPEED, 6, -6, 10);

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


           /* showMessageOnDriverStation("Move back");
            driveReverse(8, 5);

            showMessageOnDriverStation("Set arm to home");
            setArmToHome();

            showMessageOnDriverStation("Align robot to lander");
            encoderDrive(TURN_SPEED, -6, 6, 10);*/

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
            armMotor.setTargetPosition(100);
            armMotor.setPower(armSpeed);

            sleep(400);

            showMessageOnDriverStation("Move forward");
            driveForward(3.5, 5);

            sleep(500);
            elbowServo.setPosition(0.15);


          /*  showMessageOnDriverStation("Move back");
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



