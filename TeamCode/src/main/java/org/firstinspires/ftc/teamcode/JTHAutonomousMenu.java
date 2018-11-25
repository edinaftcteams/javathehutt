package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="Menu Test", group="JTH")
public class JTHAutonomousMenu extends JTHOpMode {
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
        initRobot();


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
        {
            while (opModeIsActive())
            {

                telemetry.addLine();
                telemetry.addData("Button", gamepad1.toString());
                telemetry.addData("Tank Mode", tankMode);
                telemetry.addData("isDocked", isDocked);
                telemetry.addData("isHooked", isHooked);
                telemetry.addData("Step", step);
                telemetry.update();

                unDock();

                encoderDrive(DRIVE_SPEED, 40, 40, 2.0);  // Forward 4 Inches with 2 Sec timeout

                markerServo.setPosition(0);

                encoderDrive(TURN_SPEED, -7, 7, 2.0);  // Backward 4 Inches with 2 Sec timeout

                encoderDrive(DRIVE_SPEED, 30, -30, 6.0);  // left turn 2 Inches with 1 Sec timeout

                encoderDrive(TURN_SPEED, -2, 2, 2.0);

                encoderDrive(DRIVE_SPEED, 80, 80, 5.0);

                encoderDrive(TURN_SPEED, -1, 1, 2.0);

                encoderDrive(DRIVE_SPEED, -100, -100, 6.0);
            }
        }
        else
        {
            while (opModeIsActive()) {

                telemetry.addLine();
                telemetry.addData("Button", gamepad1.toString());
                telemetry.addData("Tank Mode", tankMode);
                telemetry.addData("isDocked", isDocked);
                telemetry.addData("isHooked", isHooked);
                telemetry.addData("Step", step);
                telemetry.update();

                unDock();

                encoderDrive(DRIVE_SPEED, 75, 75, 2.0);  // Forward 4 Inches with 2 Sec timeout

                markerServo.setPosition(0);

                encoderDrive(TURN_SPEED, 10, -10, 2.0);  // Backward 4 Inches with 2 Sec timeout

                encoderDrive(DRIVE_SPEED, -120, -120, 6.0);  // left turn 2 Inches with 1 Sec timeout
                break;
            }
        }
    }
}
