package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoRedRight")
public class AutoRedRight extends StarterAuto {


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(12, -64, 0));
        zeroAngle = getCurrentPose().angle;
        long currentTime = System.nanoTime();
        long previousTime = System.nanoTime();
        boolean done = false;


        ColorDetector colorDetector = new ColorDetector(10, 10, true, hardwareMap);
        packet.put("location",colorDetector.location);
        dashboard.sendTelemetryPacket(packet);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        packet.put("location",colorDetector.location);
        dashboard.sendTelemetryPacket(packet);
        // detect the colour (positions are estimates)
        //robot is about 16 inches long
        if (colorDetector.location == Location.CENTER) {
            while (!driveToPointAsync(new Pose(12, -39, 0), true)) {
                asyncPositionCorrector();
            }
            releasePixel();
            while (!driveToPointAsync(new Pose(12, -60, 0), true)) {
                asyncPositionCorrector();
            }
            turnRobot(Math.PI/2);
            while (!driveToPointAsync(new Pose(61, -60, Math.PI/2), true)) {
                asyncPositionCorrector();
            }
            pixelPlaceAuto(Location.CENTER,true);

            // rotate and then move or spline under gate past E towards center of backdrop
                }
        else if (colorDetector.location == Location.LEFT) {
            while (!driveToPointAsync(new Pose(12, -36, 0), true)) {
                asyncPositionCorrector();
            }

            while (!driveToPointAsync(new Pose(-1.5, -40, 0), true)) {
                asyncPositionCorrector();
            }
            releasePixel();
            while (!driveToPointAsync(new Pose(12, -40, 0), true)) {
                asyncPositionCorrector();
            }
            turnRobot(Math.PI/2);
            while (!driveToPointAsync(new Pose(61, -56, Math.PI/2), true)) {
                asyncPositionCorrector();
            }
            pixelPlaceAuto(Location.LEFT,true);



        } else { //right, (-27, -45)

            while (!driveToPointAsync(new Pose(22.5, -40, 0), true)) {
                asyncPositionCorrector();
            }
            releasePixel();
            while (!driveToPointAsync(new Pose(22.5, -48, 0), true)) {
                asyncPositionCorrector();
            }
            turnRobot(Math.PI/2);
            while (!driveToPointAsync(new Pose(61, -60, Math.PI/2), true)) {
                asyncPositionCorrector();
            }
            pixelPlaceAuto(Location.RIGHT,true);

        }
    }
}
