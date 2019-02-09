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

        antiTiltThread.start();

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
            if (dockLocation == "Depo") {
                if (position == "LEFT") {
                    sampleLeftDepo();
                } else if (position == "RIGHT") {
                    sampleRightDepo();
                } else if (position == "MIDDLE") {
                    sampleMiddleDepo();
                }
            } else {
                if (position == "LEFT") {
                    sampleLeft();
                } else if (position == "RIGHT") {
                    sampleRight();
                } else if (position == "MIDDLE") {
                    sampleMiddle();
                }
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

        antiTiltThread.interrupt();

        //sleep(2000);
    }


    //Code for Crater location
    //Code for sampling

    public void sampleLeft() {
        driveForward(4, 22.2);

        turn(26, 0.3);

        driveForward(20, 22.2);

        turn(58, 0.3);

        //encoderDrive(TURN_SPEED, -27, 27, 10);

        driveForwardFast(50, 22.2);

    }

    public void sampleMiddle() {

        driveForward(22, 22.2);

        driveReverse(11, 22.2);

        turn(68, 0.3);

        driveForwardFast(25, 22.2);

        turn(15, 0.3);

        driveForwardFast(35, 22.2);

    }

    public void sampleRight() {
        driveForward(4, 22.2);

        turn(-22, 0.3);

        driveForward(18, 22.2);


        driveForward(-13, 22.2);
        //driveReverse(13, 22.2);

        turn(90, 0.4);

        driveForwardFast(28, 22.2);

        //turn(18, 0.4);

        encoderDrive(TURN_SPEED, -4, 4, 10);


        driveForwardFast(34, 22.2);
    }



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
        driveForward(-20, 10);

        setArmToHomeDown();

        // sleep(500);

        showMessageOnDriverStation("Turn around");
        encoderDrive(TURN_SPEED, -27, 27, 10);


        driveForward(28, 22.2);

        reachIntoCraterWithoutSlide();

        sleep(2000);
/*        showMessageOnDriverStation("Move forward");
        driveForward(10, 22.2);

        showMessageOnDriverStation("Reach out");
        reachIntoCrater();

        showMessageOnDriverStation("Move forward");
        driveForward(10, 22.2);*/
    }


    public void parkAllianceFromCrater() {

        //not tested

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        showMessageOnDriverStation("Drive forward");
        driveForward(7, 22.2);
    }


    //Code for Depo location

    public void sampleLeftDepo() {
        driveForward(4, 22.2);

        turn(26, 0.3);

        driveForward(30, 22.2);


    }


    public void claimFromDepoLeft() {

        turn(-20, 0.3);


        driveForward(32, 22.2);

        //sleep(300);

        driveForwardFast(-18, 22.2);

        //turn(-4,0.3);

        //driveReverse(10, 22.2);

        //turn(4,0.3);

        reachIntoCraterWithHalfSlide();

        wristServo.setPosition(0.5);

        sleep(1000);

        showMessageOnDriverStation("Drop Marker");
        dropMarker();

        //sleep(500);

        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(50);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        sleep(1000);

        setArmToHomeDown();

        sleep(500);


        //driveForwardFast(-30, 10);

        showMessageOnDriverStation("Turn around");
        encoderDrive(TURN_SPEED, 25, -25, 10);


        // showMessageOnDriverStation("Reach out");


        //sleep(2000);

        driveForwardFast(50, 22.2);


        reachIntoCraterWithoutSlide();

        sleep(2000);
    }


    public void sampleMiddleDepo() {

        turn(1, 0.3);

        driveForward(25, 22.2);

        //sleep(300);

        // driveReverse(7, 22.2);

    }


    public void claimFromDepoMiddleParallelToWall() {

        turn(22, 0.3);

        driveForward(10, 22.2);

        turn(-25, 0.3);

        driveForward(31, 22.2);

        //sleep(300);

        driveReverse(4, 22.2);

        turn(-15, 0.3);

        driveReverse(14, 22.2);

        //turn(5,0.3);

        reachIntoCraterWithHalfSlide();

        wristServo.setPosition(0.5);

        sleep(1000);

        showMessageOnDriverStation("Drop Marker");
        dropMarker();


        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(50);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        setArmToHomeDown();


        sleep(500);


        driveForward(-30, 10);

        showMessageOnDriverStation("Turn around");
        encoderDrive(TURN_SPEED, 25, -25, 10);


        showMessageOnDriverStation("Reach out");
        reachIntoCrater();


        //sleep(2000);

        driveForward(8, 22.2);

    }


    public void claimFromDepoMiddle() {

        turn(-5, 0.3);


        driveForward(5, 22.2);

        //showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        wristServo.setPosition(0.5);

        sleep(1000);

        showMessageOnDriverStation("Drop Marker");
        dropMarker();

        //sleep(500);

        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(50);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        sleep(1000);

        setArmToHomeDown();


        sleep(500);

        driveForward(12, 22.2);


        turn(95, 0.3);
        //encoderDrive(TURN_SPEED, 11, -11, 10);

        driveForward(15, 22.2);

        turn(10, 0.3);

        //encoderDriveFast(TURN_SPEED, -5, 25, 10);


        showMessageOnDriverStation("Drive forward");
        driveForward(45, 22.2);


        showMessageOnDriverStation("Reach out to crater");
        reachIntoCraterWithoutSlide();

        sleep(2000);
        //sleep(3000);
    }


    public void sampleRightDepo() {
        driveForward(4, 22.2);

        turn(-21, 0.3);

        driveForward(33, 22.2);


    }


    public void claimFromDepoRight() {

        turn(45, 0.3);

        driveForwardFast(20, 22.2);

        sleep(500);

        driveForwardFast(-13, 22.2);

        reachIntoCrater();

        wristServo.setPosition(0.5);

        sleep(1000);

        showMessageOnDriverStation("Drop Marker");
        dropMarker();


        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(50);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);


        sleep(1000);

        setArmToHomeDown();


        sleep(500);


        turn(35, 0.3);

        driveForwardFast(25, 10);

        turn(20, 0.3);


        driveForwardFast(55, 10);

        //showMessageOnDriverStation("Reach out");
        reachIntoCraterWithoutSlide();


        sleep(2000);

        // driveForwardFast(15, 22.2);

    }



    public void parkAllianceFromDepo() {




    }

    public void parkOpponentFromDepo() {


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
        armMotor.setTargetPosition(600);
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



