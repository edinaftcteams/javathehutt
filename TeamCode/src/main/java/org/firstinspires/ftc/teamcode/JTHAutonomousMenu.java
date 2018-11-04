package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Encoder Long", group="JTH")
public class JTHAutonomousMenu extends LinearOpMode {
    private JTHRobot robot = null;

    private AutonomousConfiguration autoConfig;
    // The properties are available after the
    // call to the ShowMenu method of the AutonomousConfiguration class.
    private AutonomousConfiguration.AllianceColor alliance;
    private AutonomousConfiguration.StartPosition startPosition;
    private boolean startLatched;
    private boolean sampleGold;
    private boolean placeTeamMarker;
    private boolean stopParked;
    private int startDelay = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new JTHRobot(hardwareMap, telemetry);

        // Get configuration selections from the driver.
        autoConfig = new AutonomousConfiguration(gamepad1, telemetry);
        autoConfig.ShowMenu();

        // Save the driver selections for use in the autonomous strategy.
        alliance = autoConfig.Alliance();
        startPosition = autoConfig.StartPosition();
        startLatched = autoConfig.StartLatched();
        sampleGold = autoConfig.SampleGold();
        placeTeamMarker = autoConfig.PlaceTeamMarker();
        stopParked = autoConfig.StopParked();
        startDelay = autoConfig.StartDelay();

        telemetry.clear();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start position", startPosition);
        telemetry.addData("Start latched", startLatched);
        telemetry.addData("Sample gold", sampleGold);
        telemetry.addData("Place team marker", placeTeamMarker);
        telemetry.addData("Stop parked", stopParked);
        telemetry.addData("Start delay", startDelay);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        if( startPosition.equals(AutonomousConfiguration.StartPosition.Left))
            robot.runAutoTwo();
        else
            robot.runAutonomous();
    }
}
