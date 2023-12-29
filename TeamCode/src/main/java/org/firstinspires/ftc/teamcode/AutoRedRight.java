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
        initialize(new Pose(12,-64,0));
        //zeroAngle = getCurrentPose().angle;
        ColorDetector colorDetector = new ColorDetector(10, 10, true, hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        if(!opModeIsActive()){
            return;
        }
        colorDetector.closeCamera();
        dashboard.sendTelemetryPacket(packet);
        if (colorDetector.location == Location.CENTER) {
            while ((!driveToPointAsync(new Pose(12, -39, 0), true))&&opModeIsActive()) {
                asyncPositionCorrector();
            }
            releasePixel();
            while ((!driveToPointAsync(new Pose(12, -60, 0), false))&&opModeIsActive()) {
                asyncPositionCorrector();
            }
            while ((!driveToPointAsync(new Pose(61, -61, Math.PI/2), true))&&opModeIsActive()) {
                asyncPositionCorrector();
                if(armAsync(0.6)){
                    stringAsync(VOLTSSTRINGUP);
                }
            }
            pixelPlaceAuto(Location.CENTER,true);
            sleep(400);
            returnArm();


            // rotate and then move or spline under gate past E towards center of backdrop
                }
        else if (colorDetector.location == Location.LEFT) {
            while ((!driveToPointAsync(new Pose(12, -36, 0), false))&&opModeIsActive()) {
                asyncPositionCorrector();
            }

            while ((!driveToPointAsync(new Pose(0, -40, 0), true))&&opModeIsActive()) {
                asyncPositionCorrector();
            }
            releasePixel();
            while ((!driveToPointAsync(new Pose(12, -40, 0), false))&&opModeIsActive()) {
                asyncPositionCorrector();
            }
            while ((!driveToPointAsync(new Pose(40, -60,0), false))&&opModeIsActive()) {
                asyncPositionCorrector();
            }
            while ((!driveToPointAsync(new Pose(61, -61,Math.PI/2), true))&&opModeIsActive()) {
                asyncPositionCorrector();
                if(armAsync(0.6)){
                    stringAsync(VOLTSSTRINGUP);
                }
            }
            pixelPlaceAuto(Location.LEFT,true);
            sleep(400);
            returnArm();




        } else { //right, (-27, -45)

            while ((!driveToPointAsync(new Pose(24, -40, 0), true))&&opModeIsActive()) {
                asyncPositionCorrector();
            }
            releasePixel();
            while ((!driveToPointAsync(new Pose(22.5, -48, 0), false))&&opModeIsActive()) {
                asyncPositionCorrector();
            }
            while ((!driveToPointAsync(new Pose(61, -62, Math.PI/2   ), true))&&opModeIsActive()) {
                asyncPositionCorrector();
                if(armAsync(0.6)){
                    stringAsync(VOLTSSTRINGUP);
                }
            }
            pixelPlaceAuto(Location.RIGHT,true);
            sleep(400);
            returnArm();


        }
    }
}
