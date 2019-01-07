package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Autonomous code : Docked on the lander side facing depo.
 */
@Autonomous(name = "JTH Auto Depo", group = "JTH")
@Disabled
public class JTHAutoDepo extends JTHOpMode {


    @Override
    public void runOpMode() {

        initRobot();
        telemetry.addLine("Mapped hardware");

        initArm();
        telemetry.addLine("Arm set to home");

        resetArmEncoders();
        telemetry.addLine("Arm encoders reset");

        //lowerTheHook();

        hook();


        telemetry.addLine("Press Start....");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            path();
            break;
        }
    }


    public void path() {
        showMessageOnDriverStation("Lower the robot");
        lowerTheRobot();
        sleep(200);

        showMessageOnDriverStation("Unhook the robot");
        unHook();
        sleep(200);


        //moveArmUp();


        showMessageOnDriverStation("D1  : Drive Forward 11");
        driveForward(11, 5);
        sleep(200);

        showMessageOnDriverStation("D2  : Turn Left 80°");
        turnLeft(80);
        //encoderDrive(turnSpeed, 11, -11, 3);
        sleep(200);

        showMessageOnDriverStation("D3  : Drive Forward 13");
        driveForward(13, 5);
        sleep(200);

        showMessageOnDriverStation("D4  : Turn Right 43°");
        turnRight(43);
        //encoderDrive(turnSpeed, -6, 6, 3);
        sleep(200);

        showMessageOnDriverStation("D5  : Drive Forward 11");
        driveForward(11, 5);
        sleep(200);

        showMessageOnDriverStation("D6  : Turn Right 85°");
        turnRight(85);
        //encoderDrive(turnSpeed, -6, 6, 3);
        sleep(200);

        showMessageOnDriverStation("D7  : Drive Forward 24");
        driveForward(24, 5);
        sleep(200);

        showMessageOnDriverStation("D8  : Drop Marker");
        dropMarker();
        sleep(200);

        showMessageOnDriverStation("D9  : Left Turn 188°");
        turnRight(188);
        //encoderDrive(turnSpeed, 24, -24, 3);
        sleep(200);

        showMessageOnDriverStation("D10: Drive Forward 30");
        driveForward(30, 5);
        sleep(200);

        showMessageOnDriverStation("D11: Extend Arm");
        //reachIntoCrater();
        controlArmManually = false;

        wristServo.setPosition(1);
        elbowServo.setPosition(0.437);


        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(200);
        armMotor.setPower(armSpeed);
        sleep(200);

        showMessageOnDriverStation("D12: Drive Forward 11");
        driveForward(21, 5);
        sleep(200);

    }
}



