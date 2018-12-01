package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * Autonomous code : Docked on the lander side facing depo.
 */
@Autonomous(name = "JTH Auto Depo", group = "JTH")
public class JTHAutoDepo extends JTHOpMode {


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

            showMessageOnDriverStation("D1  : Drive Forward ____");
            driveForward(20,5);
            sleep(200);

            showMessageOnDriverStation("D2  : Turn Left ____°");
            turnLeft(30);
            sleep(200);

            showMessageOnDriverStation("D3  : Drive Forward ____");
            driveForward(20,5);
            sleep(200);

            showMessageOnDriverStation("D4  : Turn Right ____°");
            turnRight(30);
            sleep(200);

            showMessageOnDriverStation("D5  : Drive Forward ____");
            driveForward(20,5);
            sleep(200);

            showMessageOnDriverStation("D6  : Turn Right ____°");
            showMessageOnDriverStation("D7  : Drive Forward ____");
            driveForward(20,5);
            sleep(200);

            showMessageOnDriverStation("D8  : Drop Marker");
            dropMarker();
            sleep(200);

            showMessageOnDriverStation("D9  : Left Turn ____°");
            turnLeft(40);
            sleep(200);

            showMessageOnDriverStation("D10: Drive Forward ____");
            driveForward(20,5);
            sleep(200);

            showMessageOnDriverStation("D11: Extend Arm");
            reachIntoCrater();
            sleep(200);

            showMessageOnDriverStation("D12: Drive Forward ____");
            driveForward(20,5);
            sleep(200);

            break;
        }
    }
}



