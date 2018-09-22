package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class JTHRobot {
    private JTHHookArm hookArm = null;
    private JTHArm arm = null;

    public JTHRobot(HardwareMap hardwareMap) {
        hookArm = new JTHHookArm(hardwareMap);
    }
    public void runAtutonomous(){
        //Drop down (10 seconds)
        hookArm.armSlideDown();
        // Slider moves down
        //Move to the side so that hook comes out
        //Turn around towards claiming
        //Claim(10 seconds)
        //Drive towards depot
        //Move arm so that it lowers marker into depot
        //Turn towards crater
        //Park in crater(10 seconds)
        //Move straight towards the crater and into it
    }

}
