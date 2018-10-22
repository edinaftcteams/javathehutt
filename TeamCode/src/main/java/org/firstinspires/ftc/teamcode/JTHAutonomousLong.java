package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Encoder Long", group="JTH")
public class JTHAutonomousLong extends LinearOpMode {
    private JTHRobot robot = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new JTHRobot(hardwareMap, telemetry);
        robot.runAutoTwo();
    }
}
