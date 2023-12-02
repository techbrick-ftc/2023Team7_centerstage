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



        ColorDetector colorDetector = new ColorDetector(10, 10, false, hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
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
                while (!driveToPointAsync(new Pose(45, 60, Math.PI), true)) {
                    asyncPositionCorrector();
                }
                turnRobot(-Math.PI/2);
                while (!driveToPointAsync(new Pose(61, 60, Math.PI/2), true)) {
                    asyncPositionCorrector();
                }
                pixelPlaceAuto(Location.CENTER,true);

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
            turnRobot(Math.PI/2);
                while (!driveToPointAsync(new Pose(61, 60, Math.PI/2), true)) {
                    asyncPositionCorrector();
                }
            pixelPlaceAuto(Location.RIGHT,true);


        }
        else{ //left, (-27, -45)

                while (!driveToPointAsync(new Pose(25.5, 40, Math.PI), true)) {
                    asyncPositionCorrector();
                }
                releasePixel();
                while (!driveToPointAsync(new Pose(25.5, 60, Math.PI), true)) {
                    asyncPositionCorrector();
                }
            turnRobot(Math.PI/2);
                while (!driveToPointAsync(new Pose(61, 60, Math.PI/2), true)) {
                    asyncPositionCorrector();
                }
            pixelPlaceAuto(Location.LEFT,true);
        }
        //turnRobot(Math.PI/2);
    }
}

