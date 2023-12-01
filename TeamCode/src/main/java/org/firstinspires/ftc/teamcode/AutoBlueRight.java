package org.firstinspires.ftc.teamcode;


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
        zeroAngle = getCurrentPose().angle;



        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        ColorDetector colorDetector = new ColorDetector(10, 10, false, hardwareMap);


        packet.put("location", colorDetector.location);
            dashboard.sendTelemetryPacket(packet);
            sleep(1000);


        //packet.put("x", 3.7);
        packet.put("location", colorDetector.location);
        dashboard.sendTelemetryPacket(packet);
        // detect the colour (positions are estimates)
        //robot is about 16 inches long
        if (colorDetector.location == Location.CENTER) {
            while(!driveToPointAsync(new Pose(-36,38,Math.PI),true)){
                asyncPositionCorrector();
            }
            releasePixel();
            while(!driveToPointAsync(new Pose(-36,39 ,Math.PI),true)){
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(-56,45 ,Math.PI),true)){
                    asyncPositionCorrector();
                }
            while(!driveToPointAsync(new Pose(-56,12,Math.PI),true)){
                asyncPositionCorrector();
            }
            while(!driveToPointAsync(new Pose(60,12,Math.PI),true)){
                asyncPositionCorrector();
            }

            // rotate and then move or spline under gate past E towards center of backdrop
       }
        else if (colorDetector.location == Location.RIGHT) {
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
            while(!driveToPointAsync(new Pose(60,12,Math.PI),true)){
               asyncPositionCorrector();
            }


       }
        else{ //left, (-27, -45)
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
            while(!driveToPointAsync(new Pose(60,12,Math.PI),true)){
                asyncPositionCorrector();
           }
        }

    }
}

