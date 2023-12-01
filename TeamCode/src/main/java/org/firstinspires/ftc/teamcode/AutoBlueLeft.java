package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoBlueLeft")
public class AutoBlueLeft extends StarterAuto {


    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
//        initialize(new Pose(12,60,Math.PI));
        initialize(new Pose(12,64,Math.PI));
        waitForStart();
        zeroAngle = getCurrentPose().angle;
        long currentTime = System.nanoTime();
        long previousTime = System.nanoTime();
        boolean done = false;
        int pos1X = 55;
        int pos1Y = 270;
        int pos2X = 440;
        int pos2Y = 245;
        int pos3X = 765;
        int pos3Y = 270;
        Point[] points = {new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y)};



        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        ColorDetector colorDetector = new ColorDetector(points, 10, 10, false, hardwareMap);

        waitForStart();

        packet.put("location", colorDetector.location);
        dashboard.sendTelemetryPacket(packet);


        //packet.put("x", 3.7);
        packet.put("location", colorDetector.location);
        dashboard.sendTelemetryPacket(packet);
        // detect the colour (positions are estimates)
        //robot is about 16 inches long
        if (colorDetector.location == Location.CENTER) {
            //release
                while (!driveToPointAsync(new Pose(12, 39, Math.PI), true)) {
                    asyncPositionCorrector();
                }
                releasePixel();
                while (!driveToPointAsync(new Pose(12, 63, Math.PI), true)) {
                    asyncPositionCorrector();
                }
                while (!driveToPointAsync(new Pose(60, 60, Math.PI), true)) {
                    asyncPositionCorrector();
                }

            // rotate and then move or spline under gate past E towards center of backdrop
        }
        else if (colorDetector.location == Location.RIGHT) {
                while (!driveToPointAsync(new Pose(12, 36, Math.PI), true)) {
                    asyncPositionCorrector();
                }

                while (!driveToPointAsync(new Pose(0, 40, Math.PI), true)) {
                    asyncPositionCorrector();
                }
                releasePixel();
                while (!driveToPointAsync(new Pose(12, 40, Math.PI), true)) {
                    asyncPositionCorrector();
                }
                while (!driveToPointAsync(new Pose(60, 60, Math.PI), true)) {
                    asyncPositionCorrector();
                }


        }
        else{ //left, (-27, -45)

                while (!driveToPointAsync(new Pose(25.5, 40, Math.PI), true)) {
                    asyncPositionCorrector();
                }
                releasePixel();
                while (!driveToPointAsync(new Pose(25.5, 60, Math.PI), true)) {
                    asyncPositionCorrector();
                }
                while (!driveToPointAsync(new Pose(60, 60, Math.PI), true)) {
                    asyncPositionCorrector();
                }
        }

        while(opModeIsActive() && !done) {

        packet.put("Field Pose",fieldPose);
            packet.put("velocity Pose",velocityPose.angle);
            dashboard.sendTelemetryPacket(packet);
        }
        while(opModeIsActive()){
            asyncPositionCorrector();
            //done = driveToPoint(new Pose(96,48,0),true);
            while(driveToPointAsync(new Pose(12,30,Math.PI),true)){
                asyncPositionCorrector();
            }
        }
        //turnRobot(Math.PI/2);
    }
}

