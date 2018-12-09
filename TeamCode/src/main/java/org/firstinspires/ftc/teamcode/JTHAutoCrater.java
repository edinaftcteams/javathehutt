package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Driver;

/*
 * Autonomous code : Docked on the lander side facing crater.
 */
@Autonomous(name = "JTH Auto Crater", group = "JTH")
public class JTHAutoCrater extends JTHOpMode {


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
            sleep(7000);

            showMessageOnDriverStation("Lower the robot");
            lowerTheRobot();
            sleep(200);

            showMessageOnDriverStation("Unhook the robot");
            unHook();
            sleep(200);


            sleep(200);
            moveArmUp();

            showMessageOnDriverStation("C1  : Drive Forward 11");
            driveForward(11, 5.0);
            sleep(200);


            showMessageOnDriverStation("C2  : Turn Left 80°");
            turnLeft(80);
            sleep(200);

            showMessageOnDriverStation("C3  : Drive Forward 34");
            driveForward(33, 5.0);
            sleep(200);

            showMessageOnDriverStation("C4  : Turn Left 45°");
            turnLeft(45);
            sleep(200);

            showMessageOnDriverStation("C5  : Drive Forward 27");
            driveForward(27, 5.0);
            sleep(200);

            showMessageOnDriverStation("C6  : Drop Marker");
            dropMarker();
            sleep(200);

            showMessageOnDriverStation("C7  : Right Turn 185°");
            turnLeft(185);
            sleep(200);

            showMessageOnDriverStation("C8  :Drive Forward 30");
            driveForward(30, 5.0);
            sleep(200);

            showMessageOnDriverStation("C9  :Extend Arm");
            //reachIntoCrater();
            controlArmManually = false;

            wristServo.setPosition(1);
            elbowServo.setPosition(0.437);


            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(200);
            armMotor.setPower(armSpeed);
            sleep(200);

            showMessageOnDriverStation("C10: Drive Forward 14");
            driveForward(24, 5.0);
            sleep(200);

            break;
        }
    }
}



