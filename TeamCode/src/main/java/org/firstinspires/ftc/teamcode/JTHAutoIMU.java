package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Auto : Configurable autonomous paths.
 */
@Autonomous(name = "JTH Auto IMU", group = "JTH")
public class JTHAutoIMU extends JTHOpModeIMU {

    private boolean phoneInLandscape = true;

    @Override
    public void runOpMode() {

        String dockLocation = "Crater";
        boolean sample = true;
        boolean claim = true;
        boolean park = true;
        String parkLocation = "Alliance";
        int sleepTimer = 0;

        boolean dpad_down = false;
        boolean dpad_up = false;
        boolean dpad_left = false;
        boolean dpad_right = false;

        String position = "";
        String lastPosition = "";
        int numberOfSampleCheck = 0;


        initRobot();
        telemetry.addLine("Mapped hardware");

        initArm();
        telemetry.addLine("Arm set to home");

        resetArmEncoders();
        telemetry.addLine("Arm encoders reset");

        enableMineralDetector();
        telemetry.addLine("Gold detector enabled");

        telemetry.update();

        while ((!opModeIsActive()) & (!isStopRequested())) {
            if (gamepad1.x) {
                dockLocation = "Crater";
                parkLocation = "Alliance";
            } else if (gamepad1.b) {
                dockLocation = "Depo";
            } else if (gamepad1.y) {
                sample = true;
            } else if (gamepad1.a) {
                sample = false;
            } else if (gamepad1.left_bumper) {
                claim = false;
            } else if (gamepad1.right_bumper) {
                claim = true;
            } else if (gamepad1.dpad_down) {
                park = false;
            } else if (gamepad1.dpad_up) {
                park = true;
            } else if (gamepad1.dpad_left) {
                parkLocation = "Alliance";
            } else if (gamepad1.dpad_right) {
                if (dockLocation == "Depo") {
                    parkLocation = "Opponent";
                } else {
                    parkLocation = "Alliance";
                }
            }


            if ((gamepad2.dpad_left == true) && (dpad_left == false)) {

                if (sleepTimer > 0) {
                    sleepTimer = sleepTimer - 1000;
                }
            } else if ((gamepad2.dpad_right == true) && (dpad_right == false)) {


                sleepTimer = sleepTimer + 1000;
            }


            if ((gamepad2.dpad_down == true) && (dpad_down == false)) {

                dropMarker();

            } else if ((gamepad2.dpad_up == true) && (dpad_up == false)) {

                controlArmManually = false;

                wristServo.setPosition(0.6);
                elbowServo.setPosition(0.3);


                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(500);
                armMotor.setPower(armSpeed);


                armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSlideMotor.setTargetPosition(590);
                armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

            }

            dpad_down = gamepad2.dpad_down;
            dpad_up = gamepad2.dpad_up;
            dpad_left = gamepad2.dpad_left;
            dpad_right = gamepad2.dpad_right;


            String pathName = "";
            int score = 30;
            if (dockLocation == "Crater") {
                pathName = pathName + 'C';
            } else {
                pathName = pathName + 'D';
            }

            if (sample == true) {
                pathName = pathName + 'S';
                score = score + 25;
            }

            if (claim == true) {
                pathName = pathName + 'C';
                score = score + 15;
            }

            if (park == true) {
                score = score + 10;
                if (parkLocation == "Alliance") {
                    pathName = pathName + "PA";
                } else {
                    pathName = pathName + "PO";
                }
            }


            telemetry.addLine("Score                  : " + score + "               Path Name  : " + pathName);
            telemetry.addLine("************************************************************");
            telemetry.addLine("Dock Location   : " + dockLocation);

            position = getGoldPosition();

            telemetry.addLine("Sampling ?        : " + sample + "            " + position + " (" + detector.getYPosition() + ")");
            telemetry.addLine("Sleep                  : " + sleepTimer);
            telemetry.addLine("Claiming ?         : " + claim);
            telemetry.addLine("Parking ?           : " + park);
            telemetry.addLine("Park Location   : " + parkLocation);
            telemetry.update();

        }

        // waitForStart();


        if (sample == true) {

            for (
                    int x = 0;
                    x < 3; x++)

            {
                sleep(100);

                numberOfSampleCheck = numberOfSampleCheck + 1;


                position = getGoldPosition();


                if ((position != lastPosition) & (lastPosition != "")) {
                    x = 0;
                }

                lastPosition = position;

                showMessageOnDriverStation(numberOfSampleCheck + ". " + position + " - " + detector.getXPosition());
            }
        }

        disableMineralDetector();

        //showMessageOnDriverStation("Lower the robot");
        lowerTheRobot();


        //showMessageOnDriverStation("Unhook the robot");
        unHook();


        if (sample == true) {
            if (position == "LEFT") {
                sampleLeft();
            } else if (position == "RIGHT") {
                sampleRight();
            } else if (position == "MIDDLE") {
                sampleMiddle();
            }
        }

        sleep(sleepTimer);

        if (claim == true) {
            if (dockLocation == "Depo") {
                if (position == "LEFT") {
                    claimFromDepoLeft();
                } else if (position == "RIGHT") {
                    claimFromDepoRight();
                } else if (position == "MIDDLE") {
                    claimFromDepoMiddle();
                }
            } else {
                claimFromCrater();
            }
        }

        if (park == true) {
            if (dockLocation == "Depo") {
                if (parkLocation == "Alliance") {
                    parkAllianceFromDepo();
                } else {
                    parkOpponentFromDepo();
                }
            } else {
                if (claim == true) {
                    parkAllianceFromClaim();
                } else {
                    parkAllianceFromCrater();
                }
            }
        }

        //sleep(2000);
    }

    //Code for sampling

    public void sampleLeft() {
        driveForward(4, 11.1);

        turn(21, 0.3);

        driveForward(20, 11.1);

        turn(60, 0.3);

        driveForward(50, 11.1);

    }

    public void sampleMiddle() {

        driveForward(25, 11.1);

        driveReverse(12, 11.1);

        turn(75, 0.3);

        driveForward(25, 11.1);

        turn(15, 0.3);

        driveForward(30, 11.1);

    }

    public void sampleRight() {
        driveForward(4, 11.1);

        turn(-22, 0.3);

        driveForward(18, 11.1);

        sleep(200);

        driveReverse(13, 11.1);

        turn(94, 0.3);

        driveForward(28, 11.1);

        turn(18, 0.3);

        driveForward(34, 11.1);
    }

    //Code for Crater location

    public void claimFromCrater() {

        reachIntoCraterWithoutSlide();

        wristServo.setPosition(0.5);

        sleep(1000);

        showMessageOnDriverStation("Drop Marker");
        dropMarker();

        //sleep(2000);
    }


    public void parkAllianceFromClaim() {

        //not tested

        //showMessageOnDriverStation("Tuck before turn");
        //setArmToHomeDown();

        showMessageOnDriverStation("Move forward");
        driveForward(-25, 10);

        showMessageOnDriverStation("Turn around");
        encoderDrive(TURN_SPEED, -23, 23, 10);


        showMessageOnDriverStation("Reach out");
        reachIntoCrater();


        //sleep(2000);

        driveForward(8, 1.1);
/*        showMessageOnDriverStation("Move forward");
        driveForward(10, 1.1);

        showMessageOnDriverStation("Reach out");
        reachIntoCrater();

        showMessageOnDriverStation("Move forward");
        driveForward(10, 1.1);*/
    }


    public void parkAllianceFromCrater() {

        //not tested

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        showMessageOnDriverStation("Drive forward");
        driveForward(7, 1.1);
    }


    //Code for Depo location

    public void claimFromDepoLeft() {

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        sleep(2000);

        showMessageOnDriverStation("Drive forward");
        driveForward(7, 1.1);


        showMessageOnDriverStation("Drop Marker");
        dropMarker();


        showMessageOnDriverStation("Move backwards");

        // for middle go back 3 more inches

        driveForward(-17, 10);

        // showMessageOnDriverStation("Tuck the arm");
        //initArm();
        //setArmToHome();


    }


    public void claimFromDepoMiddle() {

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        sleep(2000);

        showMessageOnDriverStation("Drive forward");
        driveForward(7, 1.1);


        showMessageOnDriverStation("Drop Marker");
        dropMarker();


        showMessageOnDriverStation("Move backwards");

        // for middle go back 3 more inches

        driveForward(-17, 10);

        // showMessageOnDriverStation("Tuck the arm");
        //initArm();
        //setArmToHome();


    }


    public void claimFromDepoRight() {

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        sleep(2000);

        showMessageOnDriverStation("Drive forward");
        driveForward(7, 1.1);


        showMessageOnDriverStation("Drop Marker");
        dropMarker();


        showMessageOnDriverStation("Move backwards");

        // for middle go back 3 more inches

        driveForward(-17, 10);

        // showMessageOnDriverStation("Tuck the arm");
        //initArm();
        //setArmToHome();


    }



    public void parkAllianceFromDepo() {

        setArmToHomeDown();

        sleep(1000);

        driveForward(6, 10);

        showMessageOnDriverStation("Turn right");
        encoderDrive(TURN_SPEED, -8, 8, 1.1);

        showMessageOnDriverStation("Move forward");
        driveForward(28, 1.1);

        showMessageOnDriverStation("Turn right");
        encoderDrive(TURN_SPEED, -8, 8, 1.1);

        showMessageOnDriverStation("Move forward");
        driveForward(4, 1.1);

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        sleep(2000);

        //showMessageOnDriverStation("Move forward");
        // driveForward(2, 1.1);
    }

    public void parkOpponentFromDepo() {

        setArmToHomeDown();

        sleep(1000);

        driveForward(6, 10);


        //not tested
        //We are risking hitting the lander when going to the claim
        showMessageOnDriverStation("Turn left");
        encoderDrive(TURN_SPEED, 10, -10, 1.1);

        showMessageOnDriverStation("Move forward");
        driveForward(32, 20);

        showMessageOnDriverStation("Turn left");
        encoderDrive(TURN_SPEED, 6, -6, 1.1);


        //sleep(2000);

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();


        sleep(2000);


        showMessageOnDriverStation("Move forward");
        driveForward(8, 10);

        //showMessageOnDriverStation("Drop Marker");
        //dropMarker();

        //sleep(2000);
    }


    public void dropMarker() {
        //not tested

        elbowServo.setPosition(0.4);

        showMessageOnDriverStation("Move arm down to start whipping  action");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(300);
        armMotor.setPower(armSpeed);


        wristServo.setPosition(1);

        sleep(1000);


        wristServo.setPosition(0.6);

        sleep(500);


        showMessageOnDriverStation("Move arm up");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(500);
        armMotor.setPower(armSpeed);


    }


    private String getGoldPosition() {
        if (phoneInLandscape == true) {
            if (detector.getYPosition() == 0) {
                return "NOT READY";
            } else if (detector.getYPosition() <= 150) {//going left
                return "RIGHT";
            } else if ((detector.getYPosition() > 150) & (detector.getYPosition() < 330)) {//going right
                return "MIDDLE";
            } else {
                return "LEFT";
            }
        } else {
            if (detector.getXPosition() < 100) {//going left
                return "LEFT";

            } else if (detector.getXPosition() > 400) {//going right
                return "RIGHT";
            } else {
                return "MIDDLE";
            }
        }

    }
}



