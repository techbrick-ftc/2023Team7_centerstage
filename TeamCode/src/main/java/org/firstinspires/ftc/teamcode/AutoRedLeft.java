package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoRedLeft")
public class AutoRedLeft extends StarterAuto {


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(-36,-64,0));
        //zeroAngle = getCurrentPose().angle;
        ColorDetector colorDetector = new ColorDetector(10, 10, true, hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        if(!opModeIsActive()){
            return;
        }
        colorDetector.closeCamera();
        packet.put("location",colorDetector.location);
        dashboard.sendTelemetryPacket(packet);
        if (colorDetector.location == Location.CENTER) {


                while((!driveToPointAsync(new Pose(-36,-38,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();

                }
                releasePixel();
                while((!driveToPointAsync(new Pose(-36,-39 ,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-56,-45,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-56,-8,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
            while((!driveToPointAsync(new Pose(61,-10,0),true))&&opModeIsActive()){
                asyncPositionCorrector();
            }
                while((!driveToPointAsync(new Pose(61,-11,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
            turnRobot(Math.PI/2);
            pixelPlaceAuto(Location.CENTER,false);
            sleep(1000);
            returnArm();

                // rotate and then move or spline under gate past E towards center of backdrop
            }
            else if (colorDetector.location == Location.LEFT) {
                while((!driveToPointAsync(new Pose(-48,-40,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                releasePixel();
                while((!driveToPointAsync(new Pose(-60,-40,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-60,-12,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(61,-12,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
            turnRobot(Math.PI/2);
            pixelPlaceAuto(Location.LEFT,false);
            sleep(1000);
            returnArm();

            }
            else{ //right, (-27, -45)
                while((!driveToPointAsync(new Pose(-36,-40,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-24,-40,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                releasePixel();
                while((!driveToPointAsync(new Pose(-36,-40,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-36,-12,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(61,-12,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
            turnRobot(Math.PI/2);
            pixelPlaceAuto(Location.CENTER,false);
            sleep(1000);
            returnArm();
            }
        }
    }


