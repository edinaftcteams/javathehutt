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

        telemetry.addLine("Press Start....");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.start == true) {

            } else if (gamepad1.x == true) {
                turnLeft(leftTurnAngle);
            } else if (gamepad1.b == true) {
                turnRight(rightTurnAngle);
            } else if (gamepad1.y == true) {
                driveForward(testDriveDistance,5.0);
            } else if (gamepad1.a == true) {
                driveReverse(testDriveDistance,5.0);
            } else if (gamepad1.left_bumper == true) {

            } else if (gamepad1.right_bumper == true) {

            } else if (gamepad1.dpad_down == true) {

            } else if (gamepad1.dpad_up == true) {

            } else if (gamepad1.dpad_right == true) {

            } else if (gamepad1.dpad_left == true) {

            } else if ((docking == false) && (undocking == false)) {
                driveUsingPOVMode();

                if ((gamepad2.right_stick_y != 0) || (gamepad2.right_stick_x != 0)) {
                    setArmToManualControl();
                }

                if (controlArmManually == true) {
                    armMotor.setPower(-gamepad2.right_stick_y * armSpeed);
                    armSlideMotor.setPower(gamepad2.right_stick_x * armSlideSpeed);
                }

            }
        }
    }
}



