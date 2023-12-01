package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoRedRight")
public class AutoRedRight extends StarterAuto {


    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(12, -64, 0));
        waitForStart();
        zeroAngle = getCurrentPose().angle;
        long currentTime = System.nanoTime();
        long previousTime = System.nanoTime();
        boolean done = false;


        ColorDetector colorDetector = new ColorDetector(10, 10, true, hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
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
            while (!driveToPointAsync(new Pose(60, -60, 0), true)) {
                asyncPositionCorrector();
            }

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
            while (!driveToPointAsync(new Pose(60, -60, 0), true)) {
                asyncPositionCorrector();
            }


        } else { //right, (-27, -45)

            while (!driveToPointAsync(new Pose(22.5, -40, 0), true)) {
                asyncPositionCorrector();
            }
            releasePixel();
            while (!driveToPointAsync(new Pose(60, -60, 0), true)) {
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
            done = driveToPointAsync(new Pose(96, 48, 0), true);
        }
        //turnRobot(Math.PI/2);
    }
}
