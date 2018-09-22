package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class JTHAutonomous extends LinearOpMode {
    private JTHRobot robot = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new JTHRobot(hardwareMap);
        robot.runAtutonomous();
    }
}
