package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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


            showMessageOnDriverStation("Lower the robot");
            lowerTheRobot();
            sleep(200);

            showMessageOnDriverStation("Unhook the robot");
            unHook();
            sleep(200);


            moveArmUp();

            showMessageOnDriverStation("C1  : Drive Forward 11");
            driveForward(11, 5.0);
            sleep(200);


            showMessageOnDriverStation("C2  : Turn Left 80°");
            turnLeft(80);
            sleep(200);

            showMessageOnDriverStation("C3  : Drive Forward 34");
            driveForward(34, 5.0);
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
            turnRight(185);
            sleep(200);

            showMessageOnDriverStation("C8  :Drive Forward 30");
            driveForward(30, 5.0);
            sleep(200);

            showMessageOnDriverStation("C9  :Extend Arm");
            reachIntoCrater();
            sleep(200);

            showMessageOnDriverStation("C10: Drive Forward 14");
            driveForward(14, 5.0);
            sleep(200);

            break;
        }
    }
}



