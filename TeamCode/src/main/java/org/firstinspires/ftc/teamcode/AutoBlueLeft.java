package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoBlueLeft")
public class AutoBlueLeft extends StarterAuto {


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(12,64,Math.PI));
        //zeroAngle = getCurrentPose().angle;
        ColorDetector colorDetector = new ColorDetector(10, 10, false, hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        if(!opModeIsActive()){
            return;
        }
        colorDetector.closeCamera();
        dashboard.sendTelemetryPacket(packet);
        // detect the colour (positions are estimates)
        // detect the colour (positions are estimates)
        //robot is about 16 inches long
        if (colorDetector.location == Location.CENTER) {
            //release
                while ((!driveToPointAsync(new Pose(12, 37.5, Math.PI), true))&&opModeIsActive()) {
                    asyncPositionCorrector();
                }
                releasePixel();
                while ((!driveToPointAsync(new Pose(12, 63, Math.PI), false))&&opModeIsActive()) {
                    asyncPositionCorrector();
                }
                while ((!driveToPointAsync(new Pose(45, 60, Math.PI), false))&&opModeIsActive()) {
                    asyncPositionCorrector();
                }
                while ((!driveToPointAsync(new Pose(58, 57.5, Math.PI/2), true))&&opModeIsActive()) {
                    asyncPositionCorrector();
                    if(armAsync(ARMROTATE0POSITION)){
                        stringAsync(STRINGVOLTTOP);
                    }
                }
            getReadyToPlace();
            pixelPlaceAuto(Location.CENTER,false);
            sleep(400);
            returnArm();

            // rotate and then move or spline under gate past E towards center of backdrop
        }
        else if (colorDetector.location == Location.RIGHT) {
                while ((!driveToPointAsync(new Pose(12, 36, Math.PI), false))&&opModeIsActive()) {
                    asyncPositionCorrector();
                }

                while ((!driveToPointAsync(new Pose(-1.5, 39, Math.PI), true))&&opModeIsActive()) {
                    asyncPositionCorrector();
                }
                releasePixel();
                while ((!driveToPointAsync(new Pose(12, 39 , Math.PI), false))&&opModeIsActive()) {
                    asyncPositionCorrector();
                }
                while ((!driveToPointAsync(new Pose(58, 62.5, Math.PI/2), true))&&opModeIsActive()) {
                    asyncPositionCorrector();
                }
            getReadyToPlace();
            pixelPlaceAuto(Location.RIGHT,false);
            sleep(400);
            returnArm();


        }
        else{ //left, (-27, -45)

                while ((!driveToPointAsync(new Pose(25.5, 40, Math.PI), true))&&opModeIsActive()) {
                    asyncPositionCorrector();
                }
                releasePixel();
                while ((!driveToPointAsync(new Pose(25.5, 60, Math.PI), false))&&opModeIsActive()) {
                    asyncPositionCorrector();
                }
                while ((!driveToPointAsync(new Pose(58, 57.5, Math.PI/2), true))&&opModeIsActive()) {
                    asyncPositionCorrector();
                    if(armAsync(ARMROTATE0POSITION)){
                        stringAsync(STRINGVOLTTOP);
                    }
                }
            getReadyToPlace();
            pixelPlaceAuto(Location.LEFT,false);
            sleep(400);
            returnArm();
        }
//        while((!driveToPointAsync(new Pose(40,57.5,Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        while((!driveToPointAsync(new Pose(40,11,Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        while((!driveToPointAsync(new Pose(-58.5,11,Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        turnRobot(Math.PI/8);
//        turnRobot(Math.PI/2);
//        Forward();
//        intakeMotor.setPower(.6);
//        sleep(200);
//        motorsStop();
//        while((!driveToPointAsync(new Pose(58,57.5,-Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        intakeMotor.setPower(0);
    }
}

