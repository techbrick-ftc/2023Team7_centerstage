package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoBlueLeft")
public class AutoBlueLeft extends StarterAuto {

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        // initialize(new Pose(12,60,Math.PI));
        initialize(new Pose(12, 64, Math.PI));
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
        Point[] points = { new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y) };

        // ColorDetector colorDetector = new ColorDetector(points, 10, 10, false,
        // hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        // detect the colour (positions are estimates)
        // robot is about 16 inches long
        // if (colorDetector.location == Location.CENTER) {
        if (true) {
            // release
            while (!driveToPointAsync(new Pose(12, 39, Math.PI), true)) {
                asyncPositionCorrector();
            }
            releasePixel();
            while (!driveToPointAsync(new Pose(12, 63, Math.PI), true)) {
                asyncPositionCorrector();
            }
            while (!driveToPointAsync(new Pose(60, 63, Math.PI), true)) {
                asyncPositionCorrector();
            }

            // rotate and then move or spline under gate past E towards center of backdrop
        }
        // else if (colorDetector.location == Location.RIGHT) {
        else if (true) {
            while (!driveToPointAsync(new Pose(12, 36, Math.PI), true)) {
                asyncPositionCorrector();
            }

            while (!driveToPointAsync(new Pose(-1.5, 40, Math.PI), true)) {
                asyncPositionCorrector();
            }
            releasePixel();
            while (!driveToPointAsync(new Pose(12, 40, Math.PI), true)) {
                asyncPositionCorrector();
            }
            while (!driveToPointAsync(new Pose(60, 63, Math.PI), true)) {
                asyncPositionCorrector();
            }

        } else { // left, (-27, -45)

            while (!driveToPointAsync(new Pose(22.5, 40, Math.PI), true)) {
                asyncPositionCorrector();
            }
            releasePixel();
            while (!driveToPointAsync(new Pose(60, 63, Math.PI), true)) {
                asyncPositionCorrector();
            }
        }

        while (opModeIsActive() && !done) {

            packet.put("Field Pose", fieldPose);
            packet.put("velocity Pose", velocityPose.angle);
            dashboard.sendTelemetryPacket(packet);
        }
        while (opModeIsActive()) {
            asyncPositionCorrector();
            // done = driveToPoint(new Pose(96,48,0),true);
            while (driveToPointAsync(new Pose(12, 30, Math.PI), true)) {
                asyncPositionCorrector();
            }
        }
        // turnRobot(Math.PI/2);
    }
}
