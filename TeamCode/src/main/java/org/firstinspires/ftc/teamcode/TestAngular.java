package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestAngular")
public class TestAngular extends StarterAuto {


    @Override
    public void runOpMode() {
        initialize(new Pose(0,0,0));
        waitForStart();
        zeroAngle = getCurrentPose().angle;
        boolean done = false;

        while(opModeIsActive() && !done) {
            testAngular();
        }
        //turnRobot(Math.PI/2);
    }
}