package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * 1. Two drive modes. Start button toggles between the modes. Default is Tank mode. You can see if robot is in Tank mode or not in driver station telemetry log
 *    a. Tank mode : control left motor with left stick, right motor with right stick
 *    b. POV mode  : Left stick controls forward/back word movement, like accelerator in the car. Right stick controls turns, like steering wheel.
 *    ****NOTE: Added code to gradually increase the speed to eliminate robot from flipping when direction is changed rapidly. And also to give better tracking, with this new code there won't be any
 *    wheel slippage. This is better for precision control of the robot.
 * 2. Directional pad controls lift up/down and hook left/right
 * 3. Button A: Initiates autonomous docking
 * 4. Button B: Initiates autonomous undocking
 * 5. Button X: Move robot forward 4 inches
 * 5. Button Y: Move robot reverse 4 inches
 * 6. Left Bumper: Turn left 2 inches
 * 7. Right Bumper: Turn right 2 inches
 * 8. Guide : It starts gold mineral tracking. Robot should automatically align itself to gold mineral and move towards it. ****NOTE: THIS FEATURE IS COMMENTED OUT FOR NOW****
 * 9. Gamepad 2, right joystick up and down controls arm up and down.
 * 10. Gamepad 2, right joystick left and right controls arm slide.
 * 11. Gamepad 2, d-pad controls elbow and wrist
 * 12. GamePad 2, b,x positions the arm for ball pickup and drop off
 * 13. GamePad 2, left joystick controls drive and turns
 */
@TeleOp(name = "JTH OpMode", group = "JTH")
public class JTHOpMode extends LinearOpMode {


    protected ElapsedTime runtime = new ElapsedTime();

    protected static final double COUNTS_PER_MOTOR_REV = 56;
    protected static final double DRIVE_GEAR_REDUCTION = 20.0;
    protected static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    protected static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    protected static final double DRIVE_SPEED = 1;
    protected static final double TURN_SPEED = 0.7;
    protected static final double INCH_ANGLE_RATIO = 11 / 90;
    protected static final double LIFT_POWER = 1;

    protected static final double DRIVE_SPEED_PRECISE = 0.7;
    protected static final double TURN_SPEED_PRECISE = 0.5;
    protected static final double ARM_SPEED = 0.5;
    protected static final double ARM_SLIDE_SPEED = 0.8;
    protected static final double ARM_SLIDE_HOME_SPEED = 0.5;


    protected static final double WRIST_HOME = 0.2;
    protected static final double ELBOW_HOME = 0.05;
    protected static final int ARM_MAX = 2000;
    protected static final int ARM_SLIDE_MAX = 600;


    protected DigitalChannel liftBottomSwitch;  // Hardware Device Object, 0 is bottom switch, Blue wire
    protected DigitalChannel liftTopSwitch;  // Hardware Device Object, 1 is top switch, White wire
    protected DigitalChannel armStartLimit;
    protected DigitalChannel armSlideStartLimit;


    protected DcMotor liftMotor;
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;
    protected DcMotor armMotor;
    protected DcMotor armSlideMotor;

    protected Servo hookServo;
    protected Servo markerServo;
    protected Servo elbowServo;
    protected Servo wristServo;


    protected boolean configMode = false;


    protected int step = 0;
    protected int driveStarted = 0;
    protected float lastSpeed = 0;
    protected boolean isDocked = true;
    protected boolean undocking = false;
    protected boolean docking = false;
    protected boolean isHooked = false;
    protected boolean tankMode = false;
    protected boolean controlArmManually = true;

    protected double leftPOV;
    protected double rightPOV;
    protected double drivePOV;
    protected double turnPOV;
    protected double maxPOV;


    protected double driveSpeed = DRIVE_SPEED;
    protected double turnSpeed = TURN_SPEED;
    protected double inchAngleRatio = INCH_ANGLE_RATIO;
    protected double liftPower = LIFT_POWER;
    protected double armSpeed = ARM_SPEED;
    protected double armSlideSpeed = ARM_SLIDE_SPEED;
    protected int sleepBetweenSteps = 0;
    protected double testDriveDistance = 0;
    protected int leftTurnAngle = 0;
    protected int rightTurnAngle = 0;


    protected void initRobot() {
        // get a reference to our digitalTouch object.
        liftBottomSwitch = hardwareMap.get(DigitalChannel.class, "liftBottomLimit"); // 0 is bottom switch, Blue wire
        liftTopSwitch = hardwareMap.get(DigitalChannel.class, "liftTopLimit"); // 1 is top switch, White wire

        armStartLimit = hardwareMap.get(DigitalChannel.class, "armStartLimit"); // 0 is bottom switch, Blue wire
        armSlideStartLimit = hardwareMap.get(DigitalChannel.class, "armSlideStartLimit"); // 1 is top switch, White wire


        // set the digital channel to input.
        liftBottomSwitch.setMode(DigitalChannel.Mode.INPUT);
        liftTopSwitch.setMode(DigitalChannel.Mode.INPUT);

        armStartLimit.setMode(DigitalChannel.Mode.INPUT);
        armSlideStartLimit.setMode(DigitalChannel.Mode.INPUT);


        // get a reference to motor objects.
        liftMotor = hardwareMap.get(DcMotor.class, "lift_drive");
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_drive");
        armSlideMotor = hardwareMap.get(DcMotor.class, "arm_slide_drive");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        hookServo = hardwareMap.get(Servo.class, "hook_servo");
        markerServo = hardwareMap.get(Servo.class, "marker_servo");
        elbowServo = hardwareMap.get(Servo.class, "elbow_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");

        //setArmToHome();

    }

    @Override
    public void runOpMode() {

        initRobot();
        telemetry.addLine("Mapped hardware");

        initArm();
        telemetry.addLine("Arm set to home");

        resetArmEncoders();
        telemetry.addLine("Arm encoders reset");

        telemetry.addLine("Press Start....");
        telemetry.update();
        waitForStart();

        boolean y = false;
        boolean dpad_down = false;
        boolean dpad_up = false;
        boolean dpad_left = false;
        boolean dpad_right = false;

        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            telemetry.addLine();
            telemetry.addData("Pad A: Button", gamepad1.toString());
            telemetry.addData("Pad B: Button", gamepad2.toString());


            if (configMode == true) {
                telemetry.addData("You are in config mode!!", "Can't drive");
                telemetry.addData("Editing > " + getEditVariableName(), editVariable(false, false));

                telemetry.addData("testDriveDistance", testDriveDistance);
                telemetry.addData("leftTurnAngle", leftTurnAngle);
                telemetry.addData("rightTurnAngle", rightTurnAngle);

                telemetry.addData("driveSpeed", driveSpeed);
                telemetry.addData("turnSpeed", turnSpeed);
                telemetry.addData("liftPower", liftPower);
                telemetry.addData("armSpeed", armSpeed);
                telemetry.addData("armSlideSpeed", armSlideSpeed);
                telemetry.addData("InchAngleRatio", inchAngleRatio);
                telemetry.addData("sleepBetweenSteps", sleepBetweenSteps);

                telemetry.update();


                if ((gamepad2.y == true) && (y == false)) {
                    configMode = !configMode;
                } else if ((gamepad2.dpad_down == true) && (dpad_down == false)) {
                    editVariable(true, false);
                } else if ((gamepad2.dpad_up == true) && (dpad_up == false)) {
                    editVariable(true, true);
                } else if ((gamepad2.dpad_right == true) && (dpad_right == false)) {
                    if (editVariableIndex == 9) {
                        editVariableIndex = 0;
                    } else {
                        editVariableIndex = editVariableIndex + 1;
                    }
                } else if ((gamepad2.dpad_left == true) && (dpad_left == false)) {
                    if (editVariableIndex == 0) {
                        editVariableIndex = 9;
                    } else {
                        editVariableIndex = editVariableIndex - 1;
                    }
                }

                y = gamepad2.y;
                dpad_down = gamepad2.dpad_down;
                dpad_up = gamepad2.dpad_up;
                dpad_right = gamepad2.dpad_right;
                dpad_left = gamepad2.dpad_left;


            } else {
                telemetry.addData("You are in drive mode!!", "Have fun");

                telemetry.addData("armStartLimit", armStartLimit.getState());
                telemetry.addData("armSlideStartLimit", armSlideStartLimit.getState());

                telemetry.addData("controlArmManually", controlArmManually);

                telemetry.addData("Arm Slide Position", armSlideMotor.getCurrentPosition());
                telemetry.addData("Arm Position", armMotor.getCurrentPosition());
                telemetry.addData("Elbow Position", elbowServo.getPosition());
                telemetry.addData("Wrist Position", wristServo.getPosition());


                telemetry.update();


                if (gamepad2.start == true) {
                    setArmToHome();
                }


                if (gamepad2.dpad_down == true) {
                    wristServo.setPosition(Range.clip(wristServo.getPosition() - 0.01, 0, 1));
                }

                if (gamepad2.dpad_up == true) {
                    wristServo.setPosition(Range.clip(wristServo.getPosition() + 0.01, 0, 1));
                }


                if (gamepad2.x == true) {
                    reachUptoLander();
                }

                if (gamepad2.b == true) {
                    reachIntoCrater();
                }

                if ((gamepad2.y == true) && (y == false)) {
                    configMode = !configMode;
                }

                if (gamepad2.right_bumper == true) {
                    wristServo.setPosition(0.5);
                }

                if (gamepad2.left_bumper == true) {
                    wristServo.setPosition(1);
                }

                if (gamepad2.dpad_right == true) {
                    elbowServo.setPosition(Range.clip(elbowServo.getPosition() - 0.01, 0, 1));
                }

                if (gamepad2.dpad_left == true) {
                    elbowServo.setPosition(Range.clip(elbowServo.getPosition() + 0.01, 0, 1));
                }


                if (gamepad1.start == true) {
                    //unDock();
                } else if (gamepad1.x == true) {
                    encoderDrive(TURN_SPEED_PRECISE, 2, -2, 1.0);  // left turn 2 Inches with 1 Sec timeout
                } else if (gamepad1.b == true) {
                    encoderDrive(TURN_SPEED_PRECISE, -2, 2, 1.0);  // right turn 2 Inches with 1 Sec timeout
                } else if (gamepad1.y == true) {
                    encoderDrive(DRIVE_SPEED_PRECISE, -4, -4, 2.0);  // Forward 4 Inches with 2 Sec timeout
                } else if (gamepad1.a == true) {
                    encoderDrive(DRIVE_SPEED_PRECISE, 4, 4, 2.0);  // Backward 4 Inches with 2 Sec timeout
                } else if (gamepad1.left_bumper == true) {
                    encoderDrive(turnSpeed, 2, -2, 1.0);  // left turn 2 Inches with 1 Sec timeout
                } else if (gamepad1.right_bumper == true) {
                    encoderDrive(turnSpeed, -2, 2, 1.0);  // right turn 2 Inches with 1 Sec timeout
                } else if (gamepad1.dpad_down == true) {
                    lowerTheHook();
                } else if (gamepad1.dpad_up == true) {
                    liftTheHook();
                } else if (gamepad1.dpad_right == true) {
                    hook();
                } else if (gamepad1.dpad_left == true) {
                    unHook();
                } else if ((docking == false) && (undocking == false)) {
                    driveUsingPOVMode();

                    if ((gamepad2.right_stick_y != 0) || (gamepad2.right_stick_x != 0)) {
                        setArmToManualControl();
                    }

                    if (controlArmManually == true) {
                        armMotor.setPower(-gamepad2.right_stick_y * armSpeed);
                        armSlideMotor.setPower(gamepad2.right_stick_x * armSlideSpeed);
                    }

                }

            }


        }

    }


    public void resetArmEncoders() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reachIntoCrater() {
        controlArmManually = false;

        wristServo.setPosition(1);
        elbowServo.setPosition(0.437);


        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(190);
        armMotor.setPower(armSpeed);


        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(590);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);
    }

    public void reachUptoLander() {
        controlArmManually = false;

        wristServo.setPosition(0);
        elbowServo.setPosition(0.5);


        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(ARM_MAX);
        armMotor.setPower(armSpeed);


        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(10);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);
    }

    protected int editVariableIndex = 0;

    public double editVariable(boolean edit, boolean increase) {


        switch (editVariableIndex) {
            case 0:
                if (edit) {
                    if (increase) {
                        driveSpeed = driveSpeed + 0.01;
                    } else {
                        driveSpeed = driveSpeed - 0.01;
                    }
                }
                return driveSpeed;

            case 1:

                if (edit) {
                    if (increase) {
                        turnSpeed = turnSpeed + 0.01;
                    } else {
                        turnSpeed = turnSpeed - 0.01;
                    }
                }
                return turnSpeed;
            case 2:
                if (edit) {
                    if (increase) {
                        liftPower = liftPower + 0.01;
                    } else {
                        liftPower = liftPower - 0.01;
                    }
                }
                return liftPower;
            case 3:
                if (edit) {
                    if (increase) {
                        armSpeed = armSpeed + 0.01;
                    } else {
                        armSpeed = armSpeed - 0.01;
                    }
                }
                return armSpeed;
            case 4:
                if (edit) {
                    if (increase) {
                        armSlideSpeed = armSlideSpeed + 0.01;
                    } else {
                        armSlideSpeed = armSlideSpeed - 0.01;
                    }
                }
                return armSlideSpeed;
            case 5:
                if (edit) {
                    if (increase) {
                        inchAngleRatio = inchAngleRatio + 0.001;
                    } else {
                        inchAngleRatio = inchAngleRatio - 0.001;
                    }
                }
                return inchAngleRatio;
            case 6:
                if (edit) {
                    if (increase) {
                        sleepBetweenSteps = sleepBetweenSteps + 1;
                    } else {
                        sleepBetweenSteps = sleepBetweenSteps - 1;
                    }
                }
                return sleepBetweenSteps;
            case 7:
                if (edit) {
                    if (increase) {
                        testDriveDistance = testDriveDistance + 1;
                    } else {
                        testDriveDistance = testDriveDistance - 1;
                    }
                }
                return testDriveDistance;
            case 8:
                if (edit) {
                    if (increase) {
                        leftTurnAngle = leftTurnAngle + 1;
                    } else {
                        leftTurnAngle = leftTurnAngle - 1;
                    }
                }
                return leftTurnAngle;
            case 9:
                if (edit) {
                    if (increase) {
                        rightTurnAngle = rightTurnAngle + 1;
                    } else {
                        rightTurnAngle = rightTurnAngle - 1;
                    }
                }
                return rightTurnAngle;
            default:
                break;
        }

        return 666;
    }


    public String getEditVariableName() {


        switch (editVariableIndex) {
            case 0:
                return "driveSpeed";
            case 1:
                return "turnSpeed";
            case 2:
                return "liftPower";
            case 3:
                return "armSpeed";
            case 4:
                return "armSlideSpeed";
            case 5:
                return "inchAngleRatio";
            case 6:
                return "sleepBetweenSteps";
            case 7:
                return "testDriveDistance";
            case 8:
                return "leftTurnAngle";
            case 9:
                return "rightTurnAngle";
            default:
                break;
        }
        return "Something wen't wrong";

    }

    public void driveForward(double inches, double timeoutS) {
        encoderDrive(driveSpeed, -inches, -inches, timeoutS);
    }

    public void driveReverse(double inches, double timeoutS) {
        encoderDrive(driveSpeed, inches, inches, timeoutS);
    }

    public void turnRight(int angle) {
        encoderDrive(turnSpeed, angle * inchAngleRatio, -1 * angle * inchAngleRatio, 3);
    }

    public void turnLeft(int angle) {
        encoderDrive(turnSpeed, -1 * angle * inchAngleRatio, angle * inchAngleRatio, 3);
    }


    public void initArm() {

        wristServo.setPosition(WRIST_HOME);
        elbowServo.setPosition(ELBOW_HOME);

        sleep(500);
        idle();


        while (armSlideStartLimit.getState() == true) {
            armSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armSlideMotor.setPower(-armSlideSpeed);
        }
        armSlideMotor.setPower(0);

        armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        idle();


        while (armStartLimit.getState() == true) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(-0.3);
        }
        armMotor.setPower(0);


        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void setArmToHome() {
        controlArmManually = false;

        wristServo.setPosition(WRIST_HOME);
        elbowServo.setPosition(ELBOW_HOME);

        sleep(500);
        idle();

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(10);
        armMotor.setPower(armSpeed);


        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(10);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);


    }

    public void setArmToManualControl() {
        if (controlArmManually == false) {
            controlArmManually = true;

            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //armSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void unDock() {

        if ((docking == false) && (undocking == false)) {
            undocking = true;
            lowerTheRobot();
            sleep(200);
            idle();
            unHook();
            sleep(200);
            idle();
            lowerTheHook();
            isDocked = false;

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            encoderDrive(driveSpeed, 6, 6, 5.0);  // S1: Forward 6 Inches with 5 Sec timeout
            encoderDrive(turnSpeed, 2, -2, 4.0);  // S2: Turn Right 2 Inches with 4 Sec timeout
            encoderDrive(driveSpeed, 6, 6, 4.0);  // S3: Forward 6 Inches with 4 Sec timeout
            encoderDrive(turnSpeed, -2, 2, 4.0);  // S4: Turn 2 Inches with 4 Sec timeout
            encoderDrive(driveSpeed, 40, 40, 4.0);  // S5: Forward 40 Inches with 4 Sec timeout

            undocking = false;
        }

    }

    public void dock() {
        if ((docking == false) && (undocking == false)) {
            docking = true;
            liftTheHook();
            sleep(200);
            idle();
            hook();
            sleep(2000);
            idle();
            liftTheRobot();
            isDocked = true;
        }
    }

    public boolean lowerTheRobot() {
        return liftTheHook();
    }

    public boolean liftTheHook() {
        while ((opModeIsActive()) && (liftTopSwitch.getState() == true)) {

            telemetry.addData("liftTopSwitch", "Is Not Pressed, Lifting the hook");
            telemetry.update();
            // Set the motor to the new power and pause;
            liftMotor.setPower(LIFT_POWER);
        }
        liftMotor.setPower(0);
        return true;
    }

    public boolean liftTheRobot() {
        return lowerTheHook();
    }

    public boolean lowerTheHook() {
        while ((opModeIsActive()) && (liftBottomSwitch.getState() == true)) {
            telemetry.addData("liftBottomSwitch", liftBottomSwitch.getState());
            telemetry.update();
            // Set the motor to the new power and pause;
            liftMotor.setPower(-LIFT_POWER);
        }
        telemetry.addData("liftBottomSwitch", liftBottomSwitch.getState());
        telemetry.update();
        liftMotor.setPower(0);
        return true;
    }

    public boolean hook() {
        hookServo.setPosition(0);
        isHooked = true;
        return true;
    }

    public boolean unHook() {
        hookServo.setPosition(0.5);
        isHooked = false;
        return true;
    }

    public void dropMarker() {
        markerServo.setPosition(0);
        sleep(500);
        markerServo.setPosition(1);
    }

    public void driveUsingTankMode() {
        leftMotor.setPower(gamepad1.left_stick_y * driveSpeed);
        rightMotor.setPower(gamepad1.right_stick_y * driveSpeed);
    }

    public void driveUsingPOVModePreciseGamePad2() {

        if (gamepad2.left_stick_x != 0) {
            leftMotor.setPower(-gamepad2.left_stick_x * TURN_SPEED_PRECISE);
            rightMotor.setPower(gamepad2.left_stick_x * TURN_SPEED_PRECISE);
        } else {

            drivePOV = gamepad2.left_stick_y * DRIVE_SPEED_PRECISE;
            turnPOV = -gamepad2.left_stick_x * TURN_SPEED_PRECISE;

            // Combine drive and turn for blended motion.
            leftPOV = drivePOV - turnPOV;
            rightPOV = drivePOV + turnPOV;

            // Normalize the values so neither exceed +/- 1.0
            maxPOV = Math.max(Math.abs(leftPOV), Math.abs(rightPOV));
            if (maxPOV > 1.0) {
                leftPOV /= maxPOV;
                rightPOV /= maxPOV;
            }

            // Output the safe vales to the motor drives.
            leftMotor.setPower(-leftPOV);
            rightMotor.setPower(-rightPOV);
        }
    }


    public void driveUsingPOVMode() {

        float x = gamepad1.right_stick_x;
        float y = -gamepad1.left_stick_y;
        boolean preciseDrive = false;

        if ((gamepad2.left_stick_x != 0) || (gamepad2.left_stick_y != 0)) {
            x = gamepad2.left_stick_x;
            y = -gamepad2.left_stick_y;
            preciseDrive = true;
        }

        if (y == 0) {
            if (lastSpeed != 0) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                sleep(300);
            }
            driveStarted = 0;
        } else if (((lastSpeed > 0) && (y < 0)) || ((lastSpeed < 0) && (y > 0))) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(300);
            driveStarted = 0;
        } else {
            driveStarted++;
        }

        lastSpeed = y;


        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.

        if ((driveStarted < 100) && (preciseDrive == false)) {
            drivePOV = -y * 0.01 * driveStarted * driveSpeed;
            turnPOV = -x * turnSpeed;
        } else if (preciseDrive == true) {
            drivePOV = -y * DRIVE_SPEED_PRECISE;
            turnPOV = -x * TURN_SPEED_PRECISE;
        } else {
            drivePOV = -y * driveSpeed;
            turnPOV = -x * turnSpeed;
        }


        // Combine drive and turn for blended motion.
        leftPOV = drivePOV - turnPOV;
        rightPOV = drivePOV + turnPOV;

        // Normalize the values so neither exceed +/- 1.0
        maxPOV = Math.max(Math.abs(leftPOV), Math.abs(rightPOV));
        if (maxPOV > 1.0) {
            leftPOV /= maxPOV;
            rightPOV /= maxPOV;
        }

        // Output the safe vales to the motor drives.
        leftMotor.setPower(-leftPOV);
        rightMotor.setPower(-rightPOV);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void showMessageOnDriverStation(String msg) {

        telemetry.addLine(msg);
        telemetry.update();
        sleep(sleepBetweenSteps * 1000);

    }
}
