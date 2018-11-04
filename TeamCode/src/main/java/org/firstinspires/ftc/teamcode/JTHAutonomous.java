package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="JTHAutonomous", group="Linear OpMode")
public class JTHAutonomous extends LinearOpMode {
    private JTHRobot robot = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new JTHRobot(hardwareMap, telemetry);
        robot.runAutonomous();
    }
}
