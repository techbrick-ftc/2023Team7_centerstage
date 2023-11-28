package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoBlueRight")
public class AutoBlueRight extends StarterAuto {


    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(-36,64,Math.PI));
        waitForStart();
        zeroAngle = getCurrentPose().angle;
        long currentTime = System.nanoTime();
        long previousTime = System.nanoTime();
        boolean done = false;
        int pos1X = 124;
        int pos1Y = 310;
        int pos2X = 392;
        int pos2Y = 262;
        int pos3X = 660;
        int pos3Y = 278;
        Point[] points = {new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y)};


       //ColorDetector colorDetector = new ColorDetector(points, 10, 10, false, hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        // detect the colour (positions are estimates)
        //robot is about 16 inches long
        //if (colorDetector.location == Location.CENTER) {
            if(false){
            while(!driveToPointAsync(new Pose(-36,38,Math.PI),true)){
                asyncPositionCorrector();
            }
            releasePixel();
            while(!driveToPointAsync(new Pose(-36,39 ,Math.PI),true)){
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(-56,45 ,Math.PI),true)){
                    asyncPositionCorrector();
                }
            while(!driveToPointAsync(new Pose(-56,12,Math.PI),true)){
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(60,12,Math.PI),true)){
                asyncPositionCorrector();
            }

            // rotate and then move or spline under gate past E towards center of backdrop
       }
        //else if (colorDetector.location == Location.RIGHT) {
        else if(false){
            while(!driveToPointAsync(new Pose(-48,40,Math.PI),true)){
                asyncPositionCorrector();
            }
            releasePixel();
            while(!driveToPointAsync(new Pose(-60,40,Math.PI),true)){
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(-60,12,Math.PI),true)){
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(60,12,Math.PI),true)){
               asyncPositionCorrector();
            }


       }
        else{ //left, (-27, -45)
            while(!driveToPointAsync(new Pose(-36,40,Math.PI),true)){
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(-22,40,Math.PI),true)){
                asyncPositionCorrector();
            }
            releasePixel();
            while(!driveToPointAsync(new Pose(-36,40,Math.PI),true)){
                asyncPositionCorrector();
           }
            while(!driveToPointAsync(new Pose(-36,12,Math.PI),true)){
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(60,12,Math.PI),true)){
                asyncPositionCorrector();
           }
        }

    }
}

