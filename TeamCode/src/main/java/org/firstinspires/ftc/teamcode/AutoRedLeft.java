package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoRedLeft")
public class AutoRedLeft extends StarterAuto {


    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(0,0,0));
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
//        if (colorDetector.location == Location.CENTER) {
//            while(driveToPointAsync(new Pose(-36,-30,0),true)){
//                asyncPositionCorrector();
//            }
//            //release
//            while(driveToPointAsync(new Pose(-36,-45 ,0),true)){
//                asyncPositionCorrector();
//            }
//            while(driveToPointAsync(new Pose(-56,-12,0),true)){
//                asyncPositionCorrector();
//            }
//            while(driveToPointAsync(new Pose(60,-12,0),true)){
//                asyncPositionCorrector();
//            }
//
//            // rotate and then move or spline under gate past E towards center of backdrop
//        }
//        else if (colorDetector.location == Location.LEFT) {
//            while(driveToPointAsync(new Pose(-48,-40,0),true)){
//                asyncPositionCorrector();
//            }
//            //release pixel
//            while(driveToPointAsync(new Pose(-60,-40,0),true)){
//                asyncPositionCorrector();
//            }
//            while(driveToPointAsync(new Pose(-60,-12,0),true)){
//                asyncPositionCorrector();
//            }
//            while(driveToPointAsync(new Pose(60,-12,0),true)){
//                asyncPositionCorrector();
//            }
//
//
//        }
//        else{ //right, (-27, -45)
//            while(driveToPointAsync(new Pose(-36,-40,0),true)){
//                asyncPositionCorrector();
//            }
//            while(driveToPointAsync(new Pose(-24,-40,0),true)){
//                asyncPositionCorrector();
//            }
//            //release pixel
//            while(driveToPointAsync(new Pose(-36,-40,0),true)){
//                asyncPositionCorrector();
//            }
//            while(driveToPointAsync(new Pose(-36,-12,0),true)){
//                asyncPositionCorrector();
//            }
//            while(driveToPointAsync(new Pose(60,-12,0),true)){
//                asyncPositionCorrector();
//            }
//        }

        //while(opModeIsActive() && !done) {

        packet.put("Field Pose",fieldPose);
//            packet.put("velocity Pose",velocityPose.angle);
//            dashboard.sendTelemetryPacket(packet);
        //}
        while(opModeIsActive()){
            asyncPositionCorrector();
            //done = driveToPoint(new Pose(96,48,0),true);
        }
        //turnRobot(Math.PI/2);
    }
}

