package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Test Down", group = "Test")

public class TestClassReverse extends LinearOpMode {
    private DcMotor armDrive = null;
    @Override
    public void runOpMode() {
        armDrive  = hardwareMap.get(DcMotor.class, "arm_drive");
        armDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime <= 1200)) {
            armDrive.setPower(10);


        }


    }
}
