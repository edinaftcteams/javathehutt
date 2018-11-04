package org.firstinspires.ftc.teamcode;
import com.qualcomm.*;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.*;
@Autonomous(name = "Test Up", group = "Test")

public class TestClass extends LinearOpMode {
    private DcMotor armDrive = null;
    @Override
    public void runOpMode() {
        armDrive  = hardwareMap.get(DcMotor.class, "arm_drive");
        armDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        //armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime <= 1200 ) {
            armDrive.setPower(10);
        }
        //armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
}
