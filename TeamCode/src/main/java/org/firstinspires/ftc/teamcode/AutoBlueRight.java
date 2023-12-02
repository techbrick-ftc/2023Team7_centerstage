package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Location.CENTER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "AutoBlueRight")
public class AutoBlueRight extends StarterAuto {


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(-36,64,Math.PI));
        //zeroAngle = getCurrentPose().angle;
        ColorDetector colorDetector = new ColorDetector(10, 10, false, hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        colorDetector.closeCamera();
        dashboard.sendTelemetryPacket(packet);
        // detect the colour (positions are estimates)
        //robot is about 16 inches long
        if (colorDetector.location == Location.CENTER) {
            packet.put("location", "center");
            dashboard.sendTelemetryPacket(packet);
            while(!driveToPointAsync(new Pose(-36,38,Math.PI),true)){
                asyncPositionCorrector();
            }
            releasePixel();
            while (opModeIsActive()) {
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(-36,39 ,Math.PI),true)){
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(-56,45 ,Math.PI),true)){
                    asyncPositionCorrector();
                }
            while(!driveToPointAsync(new Pose(-56,12,Math.PI),true)){
                asyncPositionCorrector();
            }
            //turnRobot(-Math.PI/2);
            while(!driveToPointAsync(new Pose(61,12,Math.PI/2),true)){
                asyncPositionCorrector();
            }
            pixelPlaceAuto(Location.CENTER,false);

            // rotate and then move or spline under gate past E towards center of backdrop
       }
        else if (colorDetector.location == Location.RIGHT) {
            packet.put("location", "right");
            dashboard.sendTelemetryPacket(packet);
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
            turnRobot(Math.PI/2);
            while(opModeIsActive()){asyncPositionCorrector();}
            while(!driveToPointAsync(new Pose(61,12,Math.PI/2),true)){
               asyncPositionCorrector();
            }
            pixelPlaceAuto(Location.RIGHT,false);


       }
        else{ //left, (-27, -45)
            packet.put("location", "left");
            dashboard.sendTelemetryPacket(packet);
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
            turnRobot(Math.PI/2);
            while(!driveToPointAsync(new Pose(61,12,Math.PI/2),true)){
                asyncPositionCorrector();
           }
            pixelPlaceAuto(Location.LEFT,false);
        }

    }
}

