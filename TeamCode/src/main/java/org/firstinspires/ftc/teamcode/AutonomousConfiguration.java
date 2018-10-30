package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ron on 11/16/2016.
 * Modified: 9/24/2018
 * This class provides configuration for the autonomous opmode.
 */

public class AutonomousConfiguration {

    public AutonomousConfiguration(Gamepad gamepad, Telemetry telemetry1) {
        this.gamePad1 = gamepad;
        this.telemetry = telemetry1;
        // Default selections if driver does not select any.
        alliance = AllianceColor.None;
        startPosition = StartPosition.Left;
        startLatched = false;
        sampleGold = false;
        placeTeamMarker = false;
        stopParked = false;
        startDelay = 0;
    }

    private AllianceColor alliance;
    private StartPosition startPosition;
    private boolean startLatched;
    private boolean sampleGold;
    private boolean placeTeamMarker;
    private boolean stopParked;
    private Gamepad gamePad1;
    // Seconds to delay before starting opmode.
    private int startDelay;

    // Where do we start the robot
    public enum StartPosition {
        Left,
        Right;

        public StartPosition getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    public enum AllianceColor {
        None,
        Red,
        Blue
    }

    private Telemetry telemetry;

    public int StartDelay() {
        return startDelay;
    }

    public AllianceColor Alliance() {
        return alliance;
    }

    public StartPosition StartPosition() {
        return startPosition;
    }

    public boolean StartLatched() {
        return startLatched;
    }

    public boolean SampleGold() {
        return sampleGold;
    }

    public boolean PlaceTeamMarker() {
        return placeTeamMarker;
    }

    public boolean StopParked() {
        return stopParked;
    }

    public void ShowMenu() {
        ElapsedTime runTime = new ElapsedTime();
        // Use these to control pad press timing.
        boolean bCurrStateY = false;
        boolean bPrevStateY = false;
        boolean bCurrStateA = false;
        boolean bPrevStateA = false;

        telemetry.setAutoClear(false);
        Telemetry.Item teleAlliance = telemetry.addData("X = Blue, B = Red", Alliance());
        Telemetry.Item teleStartDelay = telemetry.addData("X decrease, B increase start delay", StartDelay());
        Telemetry.Item teleStartPosition = telemetry.addData("D-pad left/right, select start position", StartPosition());
        Telemetry.Item teleStartLatched = telemetry.addData("D-pad up to cycle start latched", StartLatched());
        Telemetry.Item teleStopParked = telemetry.addData("D-pad down to cycle stop parked", StopParked());
        Telemetry.Item teleSampleGold = telemetry.addData("Left Bumper to cycle sample gold", SampleGold());
        Telemetry.Item telePlaceTeamMarker = telemetry.addData("Right bumper to cycle place team marker", PlaceTeamMarker());
        telemetry.addData("Finished", "Press game pad Start");

        // Loop while driver makes selections.
        do {
            if (gamePad1.x) {
                alliance = AllianceColor.Blue;
            }

            if (gamePad1.b) {
                alliance = AllianceColor.Red;
            }

            teleAlliance.setValue(alliance);

            assert bCurrStateY = gamePad1.y;
            if ((bCurrStateY && (bCurrStateY != bPrevStateY))
                    && (startDelay > 0)) {
                startDelay++;
            }

            bPrevStateY = bCurrStateY;

            bCurrStateA = gamePad1.a;
            if ((bCurrStateA && (bCurrStateA != bPrevStateA))
                    && (startDelay < 20)) {
                startDelay--;
            }

            bPrevStateA = bCurrStateA;
            teleStartDelay.setValue(startDelay);

            if (gamePad1.dpad_left) {
                startPosition = StartPosition.Left;
            }

            if (gamePad1.dpad_right) {
                startPosition = StartPosition.Right;
            }

            teleStartPosition.setValue(startPosition);

            if (gamePad1.dpad_up) {
                startLatched = !startLatched;
            }

            teleStartLatched.setValue(startLatched);

            if (gamePad1.dpad_down) {
                stopParked = !stopParked;
            }

            teleStopParked.setValue(stopParked);

            if (gamePad1.left_bumper) {
                sampleGold = !sampleGold;
            }

            teleSampleGold.setValue(sampleGold);

            if (gamePad1.right_bumper) {
                placeTeamMarker = !placeTeamMarker;
            }

            telePlaceTeamMarker.setValue(placeTeamMarker);
            telemetry.update();

            // If there is no gamepad timeout for debugging.
            if (gamePad1.id == -1) {
                // The timer is for debugging, remove it when you have a gamepad connected.
                if (runTime.seconds() > 5) {
                    break;
                }
            } else {
                // Only allow loop exit if alliance has been selected.
                if (gamePad1.start && alliance != AllianceColor.None) {
                    break;
                }
            }
        } while (true);
    }
}
