package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

public class JTHHookArm {
    private DcMotor armSlide = null;
    public JTHHookArm(HardwareMap hardwareMap){
        armSlide = hardwareMap.get(DcMotor.class, "arm_slide");
    }

    public void armSlideUp() {
        armSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        armSlide.setPower(10);
    }
    public void armSlideDown() {
        armSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        armSlide.setPower(10);
    }
}
