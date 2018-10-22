package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class JTHRobot {
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    private JTHHookArm hookArm;
    private JTHArm arm;

    public DcMotor getLeftDrive() {
        return leftDrive;
    }

    public void setLeftDrive(DcMotor leftDrive) {
        this.leftDrive = leftDrive;
    }

    private DcMotor leftDrive;

    public DcMotor getRightDrive() {
        return rightDrive;
    }

    public void setRightDrive(DcMotor rightDrive) {
        this.rightDrive = rightDrive;
    }

    private DcMotor rightDrive;

    public JTHRobot(HardwareMap hardwareMap, Telemetry t) {
        //hookArm = new JTHHookArm(hardwareMap);
        telemetry = t;
        leftDrive = hardwareMap.get(DcMotor.class, JTHConstants.leftDrive);
        rightDrive = hardwareMap.get(DcMotor.class, JTHConstants.rightDrive);
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void runAutonomous(){
        //Drop down (10 seconds)
        //hookArm.armSlideDown();
        // Slider moves down
        //Move to the side so that hook comes out
        //Turn around towards claiming
        //Claim(1st position)
        this.driveDistance(100, -82, -82, 2);
        //Move arm so that it lowers marker into depot
        this.driveDistance(100, -10, 10, 5);
        //Turn towards crater
        this.driveDistance(100, 50, 50, 1);
        //Park in crater(10 seconds)
        //this.turnRobot(45);
        this.driveDistance(100, -20, 20, 2);

        this.driveDistance(100, 30, 30, 5);
        this.driveDistance(100, -8, 8, 2);
        this.driveDistance(100, -100, -100, 5);

    }
    public void driveDistance(double speed,
                              double leftInches, double rightInches,
                              double timeoutS){
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * JTHConstants.COUNTS_PER_INCH);
        newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * JTHConstants.COUNTS_PER_INCH);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while   (runtime.seconds() < timeoutS &&
                (leftDrive.isBusy() && rightDrive.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            telemetry.update();
        }



        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }
    public void turnRobot(int degrees) {
        //use time and if that is to difficult use distance
        leftDrive.setPower(10);
        rightDrive.setPower(5);
    }
    public void runAutoTwo(){
        this.driveDistance(100, -30, -30, 3);
        this.driveDistance(100, 10, -10, 3);
        this.driveDistance(100, -100, -100, 5);
        this.driveDistance(100, -13, 13, 3);
        this.driveDistance(100, -70, -70, 3);
        this.driveDistance(100, -22, 22, 3);
        this.driveDistance(100, 150, 150, 5);
        }

}
