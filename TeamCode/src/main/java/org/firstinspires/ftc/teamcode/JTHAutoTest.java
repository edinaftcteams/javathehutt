package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        while (opModeIsActive()) {

            telemetry.addData("Gold position", detector.getXPosition());
            telemetry.update();



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



