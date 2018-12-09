package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.sql.Driver;

/*
 * Test code : To test drive and turn accuracy.
 */
@TeleOp(name = "JTH Auto Test", group = "JTH")
public class JTHAutoTest extends JTHOpMode {


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


        telemetry.addLine("Press Start....");
        telemetry.update();

        waitForStart();
        telemetry.addData("Gold position", detector.getXPosition());
        telemetry.update();

        controlArmManually = false;



        reachIntoCrater();

        wristServo.setPosition(0.7585);
        elbowServo.setPosition(0.3766);



        /*armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(65);
        armMotor.setPower(armSpeed);
        armSlideMotor.setTargetPosition(313);*/
        if (detector.getXPosition() < 100){//going left
            encoderDrive(TURN_SPEED, 6, -6, 10);
            encoderDrive(DRIVE_SPEED, 8, 8, 10);
        }
        else if (detector.getXPosition() > 400){//going right
            encoderDrive(TURN_SPEED, -6, 6, 10);
            encoderDrive(DRIVE_SPEED, 8, 8, 10);
        }
        else {
            encoderDrive(DRIVE_SPEED, 3.75, 3.75, 10);
        }
        sleep(8000);
        setArmToHome();

        while (opModeIsActive()) {



            if (gamepad1.start) {

            } else if (gamepad1.x) {
                turnLeft(leftTurnAngle);
            } else if (gamepad1.b) {
                turnRight(rightTurnAngle);
            } else if (gamepad1.y) {
                driveForward(testDriveDistance,5.0);
            } else if (gamepad1.a) {
                driveReverse(testDriveDistance,5.0);
            } else if (gamepad1.left_bumper) {

            } else if (gamepad1.right_bumper) {

            } else if (gamepad1.dpad_down) {

            } else if (gamepad1.dpad_up) {

            } else if (gamepad1.dpad_right) {

            } else if (gamepad1.dpad_left) {

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



