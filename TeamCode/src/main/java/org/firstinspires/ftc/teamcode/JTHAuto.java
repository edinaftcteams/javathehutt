package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Auto : Configurable autonomous paths.
 */
@TeleOp(name = "JTH Auto", group = "JTH")
public class JTHAuto extends JTHOpMode {

    private boolean phoneInLandscape = true;

    @Override
    public void runOpMode() {

        String dockLocation = "Crater";
        boolean sample = false;
        boolean claim = false;
        boolean park = false;
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

                //reachIntoCrater();
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

            telemetry.addLine("Sampling ?        : " + sample + "            " + position + " (" + detector.getXPosition() + "," + detector.getYPosition() + ")");
            telemetry.addLine("Sleep                  : " + sleepTimer);
            telemetry.addLine("Claiming ?         : " + claim);
            telemetry.addLine("Parking ?           : " + park);
            telemetry.addLine("Park Location   : " + parkLocation);
            telemetry.update();

        }


        sleep(2000);

        telemetry.addData("Gold X position", detector.getXPosition());


        telemetry.addLine("Press Start....");
        telemetry.update();


        waitForStart();


        if (sample == true) {
            position = getGoldPosition();
          /*  for (
                    int x = 0;
                    x < 3; x++)

            {
                // sleep(1000);

                numberOfSampleCheck = numberOfSampleCheck + 1;

                if (detector.getXPosition() < 100) {//going left
                    position = "LEFT";

                } else if (detector.getXPosition() > 400) {//going right
                    position = "RIGHT";
                } else {
                    position = "MIDDLE";
                }


                if ((position != lastPosition) & (lastPosition != "")) {
                    x = 0;
                }

                lastPosition = position;

                showMessageOnDriverStation(numberOfSampleCheck + ". " + position + " - " + detector.getXPosition());
            }*/
        }

        showMessageOnDriverStation("Lower the robot");

        lowerTheRobot();


        showMessageOnDriverStation("Unhook the robot");

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
                claimFromDepo();
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
    }

    public void sampleLeft() {
        showMessageOnDriverStation("Move forward");
        driveForward(3, 1.1);

        showMessageOnDriverStation("Turn left");
        encoderDrive(TURN_SPEED, 4, -3, 1.1);

        showMessageOnDriverStation("Reach out to untuck arm");
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(100);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);


        //sleep(400);
        showMessageOnDriverStation("Move arm up");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(400);
        armMotor.setPower(armSpeed);

        //sleep(400);
        showMessageOnDriverStation("Set wrist and elbow position");
        wristServo.setPosition(0.7585);
        elbowServo.setPosition(0.1);

        showMessageOnDriverStation("reach out to mineral");
        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(590);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        //sleep(400);
        showMessageOnDriverStation("Move arm down to mineral level");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(70);
        armMotor.setPower(armSpeed);

        //sleep(400);
        showMessageOnDriverStation("Move forward");
        driveForward(4.5, 1.1);

        //sleep(500);
        showMessageOnDriverStation("Take golf shot");
        elbowServo.setPosition(0.6);

        //sampling completed, go back to home position

        //sleep(200);
        showMessageOnDriverStation("Reset elbow");
        elbowServo.setPosition(0.15);

        showMessageOnDriverStation("Init arm");
        initArm();

        showMessageOnDriverStation("Move back");
        driveForward(-6.5, 1.1);

        //sleep(200);
        showMessageOnDriverStation("Turn back to middle");
        encoderDrive(TURN_SPEED, -4, 4.5, 1.1);

        //sleep(200);
        showMessageOnDriverStation("Move farword");
        driveForward(5, 1.1);


    }

    public void sampleRight() {
        showMessageOnDriverStation("Move forward");
        driveForward(3, 1.1);

        showMessageOnDriverStation("Turn right");
        encoderDrive(TURN_SPEED, -3, 4.5, 1.1);

        showMessageOnDriverStation("Reach out to move gold mineral");
        //reachOutToMoveGoldMineral();

        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(100);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        sleep(400);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(400);
        armMotor.setPower(armSpeed);

        sleep(400);

        wristServo.setPosition(0.8585);
        //elbowServo.setPosition(0.3766);


        elbowServo.setPosition(0.6);

        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(590);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        sleep(400);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(100);
        armMotor.setPower(armSpeed);

        sleep(400);

        //showMessageOnDriverStation("Move forward");
        driveForward(1.5, 1.1);

        sleep(500);
        elbowServo.setPosition(0.15);

        sleep(200);
        initArm();

        //Sampling complete


        showMessageOnDriverStation("Turn back to middle");
        encoderDrive(TURN_SPEED, 4, -4, 1.1);

        sleep(200);
        driveForward(4, 1.1);

    }

    public void sampleMiddle() {
        showMessageOnDriverStation("Reach out to move gold mineral");
        //reachOutToMoveGoldMineral();

        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(100);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        sleep(400);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(400);
        armMotor.setPower(armSpeed);

        sleep(400);

        wristServo.setPosition(0.8585);
        //elbowServo.setPosition(0.3766);


        elbowServo.setPosition(0.6);

        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(590);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        sleep(400);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(60);
        armMotor.setPower(armSpeed);

        sleep(400);

        showMessageOnDriverStation("Move forward");
        driveForward(3.5, 1.1);

        sleep(500);
        elbowServo.setPosition(0.15);

        sleep(200);
        initArm();

        sleep(200);
        driveForward(5, 1.1);

    }

    public void parkAllianceFromCrater() {

        //not tested

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        showMessageOnDriverStation("Drive forward");
        driveForward(7, 1.1);
    }

    public void parkAllianceFromClaim() {

        //not tested

        showMessageOnDriverStation("Tuck before turn");
        initArm();

        showMessageOnDriverStation("Turn around");
        encoderDrive(TURN_SPEED, 12, -12, 1.1);

        showMessageOnDriverStation("Move forward");
        driveForward(10, 1.1);

        showMessageOnDriverStation("Reach out");
        reachIntoCrater();

        showMessageOnDriverStation("Move forward");
        driveForward(10, 1.1);
    }

    public void parkAllianceFromDepo() {

        showMessageOnDriverStation("Turn right");
        encoderDrive(TURN_SPEED, -10, 9, 1.1);

        showMessageOnDriverStation("Move forward");
        driveForward(20, 1.1);

        showMessageOnDriverStation("Turn right");
        encoderDrive(TURN_SPEED, -7, 7, 1.1);

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        showMessageOnDriverStation("Move forward");
        driveForward(18, 1.1);
    }

    public void parkOpponentFromDepo() {

        showMessageOnDriverStation("Turn left");
        encoderDrive(TURN_SPEED, 8, -8, 1.1);

        showMessageOnDriverStation("Move forward");
        driveForward(24, 1.1);

        showMessageOnDriverStation("Turn left");
        encoderDrive(TURN_SPEED, 7, -7, 1.1);

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        showMessageOnDriverStation("Move forward");
        driveForward(18, 1.1);
    }

    public void claimFromDepo() {

        //not tested

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();


        showMessageOnDriverStation("Drive forward");
        driveForward(7, 1.1);


        showMessageOnDriverStation("Drop Marker");
        dropMarker();


        showMessageOnDriverStation("Move backwards");
        driveForward(-7, 1.1);

        showMessageOnDriverStation("Tuck the arm");
        initArm();
    }

    public void claimFromCrater() {

        //not tested

        showMessageOnDriverStation("Turn left");
        encoderDrive(TURN_SPEED, 8, -8, 1.1);

        showMessageOnDriverStation("Move forward");
        driveForward(24, 1.1);

        showMessageOnDriverStation("Turn left");
        encoderDrive(TURN_SPEED, 7, -7, 1.1);

        showMessageOnDriverStation("Reach out to crater");
        reachIntoCrater();

        showMessageOnDriverStation("Move forward");
        driveForward(18, 1.1);


        showMessageOnDriverStation("Drop Marker");
        dropMarker();

    }

    public void dropMarker() {
        //not tested

        wristServo.setPosition(0.65);


        showMessageOnDriverStation("Move arm down to start whipping  action");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(320);
        armMotor.setPower(armSpeed);


        showMessageOnDriverStation("Move arm up");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(450);
        armMotor.setPower(armSpeed);

        showMessageOnDriverStation("Move arm down to start whipping  action");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(320);
        armMotor.setPower(armSpeed);


        showMessageOnDriverStation("Move arm up");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(450);
        armMotor.setPower(armSpeed);

        showMessageOnDriverStation("Move arm down to start whipping  action");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(320);
        armMotor.setPower(armSpeed);


        showMessageOnDriverStation("Move arm up");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(450);
        armMotor.setPower(armSpeed);


       /* showMessageOnDriverStation("Move arm down again");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(450);
        armMotor.setPower(armSpeed);


        showMessageOnDriverStation("Move arm up again");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(750);
        armMotor.setPower(armSpeed);


       showMessageOnDriverStation("Move arm down again");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(450);
        armMotor.setPower(armSpeed);


        showMessageOnDriverStation("Move arm up again");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(750);
        armMotor.setPower(armSpeed);*/
    }


    private String getGoldPosition() {
        if (phoneInLandscape == true) {
            if (detector.getYPosition() <= 150) {//going left
                return "LEFT";
            } else if ((detector.getYPosition() > 150) & (detector.getYPosition() < 330)) {//going right
                return "MIDDLE";
            } else {
                return "RIGHT";
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



