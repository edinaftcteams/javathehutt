package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/*
 * Sample code for JavaTheHUTT - 11/19/2018
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
@TeleOp(name = "JTH OpMode IMU", group = "JTH")
@Disabled
public class JTHOpModeIMU extends LinearOpMode {


    protected ElapsedTime runtime = new ElapsedTime();

    protected static final double COUNTS_PER_MOTOR_REV = 56;
    protected static final double DRIVE_GEAR_REDUCTION = 20.0;
    protected static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    protected static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    protected static final double DRIVE_SPEED = 0.9;
    protected static final double DRIVE_SPEED_REVERSE = 0.3;

    protected static final double TURN_SPEED = 0.5;
    protected static final double INCH_ANGLE_RATIO = 11 / 90;
    protected static final double LIFT_POWER = 1;

    protected static final double DRIVE_SPEED_PRECISE = 0.6;
    protected static final double TURN_SPEED_PRECISE = 0.4;
    protected static final double ARM_SPEED = 1; //0.8
    protected static final double ARM_SLIDE_SPEED = 1; //0.8
    protected static final double ARM_SLIDE_HOME_SPEED = 1; //0.5

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable


    protected static final double WRIST_HOME = 0.2;
    protected static final double ELBOW_HOME = 0.05;

    protected static final int ARM_MAX = 1900;
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


    public GoldAlignDetector detector;

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    double currentTilt;
    boolean tilted;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    Thread antiTiltThread = new AntiTiltThread();


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


        //rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);


        hookServo = hardwareMap.get(Servo.class, "hook_servo");
        markerServo = hardwareMap.get(Servo.class, "marker_servo");
        elbowServo = hardwareMap.get(Servo.class, "elbow_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");

        //setArmToHome();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

    }

    @Override
    public void runOpMode() {

        initRobot();
        telemetry.addLine("Mapped hardware");


       /* driveReverse(11, 5);
        telemetry.addLine("Drove Back a little");
        sleep(200);
       */

        initArm();
        telemetry.addLine("Arm set to home");

        resetArmEncoders();
        telemetry.addLine("Arm encoders reset");

        enableMineralDetector();
        telemetry.addLine("Gold detector enabled");

        telemetry.addLine("Press Start....");
        telemetry.update();



        // Set up our telemetry dashboard
        composeTelemetry();

        waitForStart();

        boolean y = false;
        boolean dpad_down = false;
        boolean dpad_up = false;
        boolean dpad_left = false;
        boolean dpad_right = false;


        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        antiTiltThread.start();

        //driveForward(100);

        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            telemetry.update();

            // correction = checkDirection();


            if (gamepad2.start == true) {
                //setArmToHome();
                setArmToHomeDown();
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
                armMotor.setTargetPosition(300);
                armMotor.setPower(armSpeed);
            }


            if (gamepad2.y == true) {
                moveArmHigherThanCrater();
            }

            if (gamepad2.a == true) {
                moveArmDown();
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


            if (gamepad1.x == true) {
                encoderDrive(TURN_SPEED_PRECISE, 2, -2, 1.0);  // left turn 2 Inches with 1 Sec timeout

                driveForward(100, 10);

            }

            if (gamepad1.b == true) {
                reachUptoLanderArmUp();
                //encoderDrive(TURN_SPEED_PRECISE, -2, 2, 1.0);  // right turn 2 Inches with 1 Sec timeout
            }
            if (gamepad1.y == true) {
                encoderDrive(DRIVE_SPEED_PRECISE, -4, -4, 2.0);  // Forward 4 Inches with 2 Sec timeout
            }
            if (gamepad1.a == true) {
                encoderDrive(DRIVE_SPEED_PRECISE, 4, 4, 2.0);  // Backward 4 Inches with 2 Sec timeout
            }
            if (gamepad1.left_bumper == true) {
                //encoderDrive(turnSpeed, 2, -2, 1.0);  // left turn 2 Inches with 1 Sec timeout
                turn(80, 0.4);
            }
            if (gamepad1.right_bumper == true) {
                //encoderDrive(turnSpeed, -2, 2, 1.0);  // right turn 2 Inches with 1 Sec timeout
                turn(-80, 0.4);
            }
            if (gamepad1.dpad_down == true) {
                lowerTheHook();
            }
            if (gamepad1.dpad_up == true) {
                liftTheHook();
            }
            if (gamepad1.dpad_right == true) {
                hook();
            }
            if (gamepad1.dpad_left == true) {
                unHook();
            }

            if (gamepad1.right_trigger != 0) {

            }

            if (gamepad1.right_trigger != 0) {

            }

            driveUsingPOVMode();

            if ((gamepad2.right_stick_y != 0) || (gamepad2.right_stick_x != 0)) {
                setArmToManualControl();
            }

            if (controlArmManually == true) {
                if ((armMotor.getCurrentPosition() > 1700) && (gamepad2.right_stick_y < 0)) {

                } else {
                    armMotor.setPower(-gamepad2.right_stick_y * armSpeed);
                }
                armSlideMotor.setPower(gamepad2.right_stick_x * armSlideSpeed);
            }


        }

        antiTiltThread.interrupt();

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
            leftMotor.setPower(Math.abs(speed * 0.01));
            rightMotor.setPower(Math.abs(speed * 0.01));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            int progressiveDrive = 1;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                progressiveDrive++;
                progressiveDrive = Range.clip(progressiveDrive, 1, 100);

                currentTilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;

                if ((currentTilt <= -7) & (leftInches < 0)) {
                    //driveReverse(6, 5);
                    //  stopDriving();
                    tilted = true;
                    leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    leftMotor.setPower(0.5);
                    rightMotor.setPower(0.5);
                    sleep(100);
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);

                    return;
                } else {
                    leftMotor.setPower(Math.abs(speed * 0.01 * progressiveDrive));
                    rightMotor.setPower(Math.abs(speed * 0.01 * progressiveDrive));

                }


                //  if (tilted) {
                //       return;
                //   }
               /* // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();*/
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


    public void encoderDriveFast(double speed,
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
            leftMotor.setPower(Math.abs(speed * 0.01));
            rightMotor.setPower(Math.abs(speed * 0.01));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            int progressiveDrive = 1;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                progressiveDrive++;
                progressiveDrive = Range.clip(progressiveDrive, 1, 100);

                currentTilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;

                if ((currentTilt <= -7) & (leftInches < 0)) {
                    //driveReverse(6, 5);
                    //  stopDriving();
                    tilted = true;
                    leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    leftMotor.setPower(0.5);
                    rightMotor.setPower(0.5);
                    sleep(100);
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);

                    return;
                } else {
                    leftMotor.setPower(Math.abs(speed));
                    rightMotor.setPower(Math.abs(speed));

                }


                //  if (tilted) {
                //       return;
                //   }
               /* // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();*/
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

    public void driveForward(double inches, double timeoutS) {
        encoderDrive(DRIVE_SPEED, -inches, -inches, timeoutS);
    }

    public void driveForwardFast(double inches, double timeoutS) {
        encoderDriveFast(DRIVE_SPEED, -inches, -inches, timeoutS);
    }

    public void driveReverse(double inches, double timeoutS) {
        encoderDrive(DRIVE_SPEED_REVERSE, inches, inches, timeoutS);
    }

    public void driveForwardIMU(double t) {

        double p = 0.5;
        for (int x = 1; x <= t; x++) {
            correction = checkDirection();


            correction = Range.clip(correction, -0.05, 0.05);

            showMessageOnDriverStation(correction + " Correction");


            leftMotor.setPower(-p);
            rightMotor.setPower(-p + correction);

            sleep(100);
            idle();
            if (tilted) {
                return;
            }
        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);


    }


    public void driveForwardInInches(double inches) {

        double p = 0.5;


        double targetCounts = Math.abs(leftMotor.getCurrentPosition()) + (inches * COUNTS_PER_INCH);

        while (opModeIsActive() & (Math.abs(leftMotor.getCurrentPosition()) < targetCounts)) {
            correction = checkDirection();
            leftMotor.setPower(-p);
            rightMotor.setPower(-p + correction);
            // showMessageOnDriverStation(leftMotor.getCurrentPosition() + " Left motor " + targetCounts);
            if (tilted) {
                return;
            }
        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);


    }


    public void driveReverseIMU(double t) {

        double p = 0.5;
        for (int x = 1; x <= t; x++) {
            correction = checkDirection();
            leftMotor.setPower(p);
            rightMotor.setPower(p + correction);
            sleep(100);
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    //----------------------------------------------------------------------------------------------
    // Anti Tilt Thread
    //----------------------------------------------------------------------------------------------
    private class AntiTiltThread extends Thread {
        public AntiTiltThread() {
            this.setName("Anti Tilt Thread");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            double lastTilt = 0;
            while (!isInterrupted()) {
                currentTilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;

                if ((lastTilt <= -7) & (currentTilt > -7)) {
                    tilted = false;
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }
                if (currentTilt <= -7) {
                    tilted = true;
                    leftMotor.setPower(0.5);
                    rightMotor.setPower(0.5);
                }

                lastTilt = currentTilt;

                idle();
            }

        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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

        //wristServo.setPosition(1);
        elbowServo.setPosition(0.437);


        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(550);
        armMotor.setPower(armSpeed);


        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(590);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);
    }


    public void reachIntoCraterWithHalfSlide() {
        controlArmManually = false;

        //wristServo.setPosition(1);
        elbowServo.setPosition(0.437);


        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(550);
        armMotor.setPower(armSpeed);


        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(200);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);
    }

    public void reachIntoCraterWithoutSlide() {
        controlArmManually = false;

        //wristServo.setPosition(1);
        elbowServo.setPosition(0.437);


        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(550);
        armMotor.setPower(armSpeed);


    }


    public void reachOutToMoveGoldMineral() {
        controlArmManually = false;

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(200);
        armMotor.setPower(armSpeed);

        wristServo.setPosition(0.7585);
        elbowServo.setPosition(0.3766);

        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(590);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(170);
        armMotor.setPower(armSpeed);

    }


    public void moveArmUp() {
        controlArmManually = false;

        elbowServo.setPosition(0.437);


        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1900);
        armMotor.setPower(armSpeed);

    }

    public void moveArmHigherThanCrater() {
        controlArmManually = false;

        //elbowServo.setPosition(0.437);


        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(400);
        armMotor.setPower(armSpeed);

    }

    public void moveArmDown() {
        controlArmManually = false;

        //elbowServo.setPosition(0.437);
        wristServo.setPosition(0.7);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(100);
        armMotor.setPower(armSpeed);

    }


    public void reachUptoLanderArmUp() {
        controlArmManually = false;

        wristServo.setPosition(0);
        elbowServo.setPosition(0.437);


        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1700);
        armMotor.setPower(armSpeed);


        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(10);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);


        elbowServo.setPosition(0);
    }

    public void reachUptoLander() {
        controlArmManually = false;

        wristServo.setPosition(0);
        elbowServo.setPosition(0.437);


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


    public void turnRight(int angle) {
        encoderDrive(turnSpeed, -1 * angle * 12 / 90, 1 * angle * 12 / 90, 3);
    }

    public void turnLeft(int angle) {
        encoderDrive(turnSpeed, 1 * angle * 12 / 90, -1 * angle * 12 / 90, 3);
    }


    public void initArm() {

        armMotor.setPower(0.3);
        sleep(500);

        armMotor.setPower(0);


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

        // wristServo.setPosition(WRIST_HOME);
        //elbowServo.setPosition(ELBOW_HOME);

        wristServo.setPosition(0);
        elbowServo.setPosition(0);

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


        wristServo.setPosition(0);
        elbowServo.setPosition(0);
    }


    public void setArmToHome() {
        controlArmManually = false;

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(900);
        armMotor.setPower(armSpeed);

        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(300);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);


        sleep(2000);


    }

    public void setArmToHomeDown() {
        controlArmManually = false;

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(500);
        armMotor.setPower(armSpeed);

        sleep(500);
        idle();

        wristServo.setPosition(0);
        elbowServo.setPosition(0);

        sleep(500);
        idle();

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(15);
        armMotor.setPower(armSpeed);


        armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armSlideMotor.setTargetPosition(10);
        armSlideMotor.setPower(ARM_SLIDE_HOME_SPEED);

        wristServo.setPosition(0);
        elbowServo.setPosition(0);

        sleep(500);


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
        hookServo.setPosition(0.7);
        isHooked = true;
        return true;
    }

    public boolean unHook() {
        hookServo.setPosition(0);
        isHooked = false;
        return true;
    }

    public void driveUsingPOVMode() {

        float x = -gamepad1.right_stick_x;
        float y = gamepad1.left_stick_y;
        boolean preciseDrive = false;

        if ((gamepad2.left_stick_x != 0) || (gamepad2.left_stick_y != 0)) {
            x = -gamepad2.left_stick_x;
            y = gamepad2.left_stick_y;
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
        driveStarted = 100;
        if ((driveStarted < 100) && (preciseDrive == false)) {
            drivePOV = -y * 0.1 * driveStarted * driveSpeed;
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


    public void showMessageOnDriverStation(String msg) {

        telemetry.addLine(msg);
        telemetry.update();
        //sleep(200);

    }


    public void enableMineralDetector() {

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings


        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!
    }


    public void disableMineralDetector() {
        detector.disable(); // Start the detector!
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;


        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;


        globalAngle += deltaAngle;

        //showMessageOnDriverStation(globalAngle + " globalAngle " + deltaAngle);

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void turn(double degrees, double p) {

        power = p;

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                if (tilted) {
                    //return;
                }
                idle();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                if (tilted) {
                    //  return;
                }
                idle();
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                if (tilted) {
                    // return;
                }
                idle();
            }

        // turn the motors off.
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}