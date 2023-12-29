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
        if(!opModeIsActive()){
            return;
        }
        colorDetector.closeCamera();
        dashboard.sendTelemetryPacket(packet);
        // detect the colour (positions are estimates)
        //robot is about 16 inches long
        if (colorDetector.location == Location.CENTER) {
            packet.put("location", "center");
            dashboard.sendTelemetryPacket(packet);
            while((!driveToPointAsync(new Pose(-36,38,Math.PI),true))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            releasePixel();
            while((!driveToPointAsync(new Pose(-36,39 ,Math.PI),false))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            while((!driveToPointAsync(new Pose(-56,45 ,Math.PI),false))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
            while((!driveToPointAsync(new Pose(-56,8,Math.PI),true))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            while((!driveToPointAsync(new Pose(60,12,Math.PI/2),true))&&opModeIsActive()){
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
        else if (colorDetector.location == Location.RIGHT) {
            packet.put("location", "right");
            dashboard.sendTelemetryPacket(packet);
            while((!driveToPointAsync(new Pose(-48,40,Math.PI),true))&&opModeIsActive()){
                    asyncPositionCorrector();
            }
            releasePixel();
            while((!driveToPointAsync(new Pose(-60,40,Math.PI),false))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            while((!driveToPointAsync(new Pose(-60,8,Math.PI),false))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            while((!driveToPointAsync(new Pose(60,8,Math.PI),true))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            while((!driveToPointAsync(new Pose(60,10,Math.PI/2),true))&&opModeIsActive()){
                asyncPositionCorrector();
                if(armAsync(0.6)){
                    stringAsync(VOLTSSTRINGUP);
                }
            }
            pixelPlaceAuto(Location.RIGHT,true);
            sleep(400);
            returnArm();


       }
        else{ //left, (-27, -45)
            packet.put("location", "left");
            dashboard.sendTelemetryPacket(packet);
            while((!driveToPointAsync(new Pose(-36,39,Math.PI),false))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            while((!driveToPointAsync(new Pose(-23,39,Math.PI),true))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            releasePixel();
            while((!driveToPointAsync(new Pose(-36,39,Math.PI),false))&&opModeIsActive()){
                asyncPositionCorrector();
           }
            while((!driveToPointAsync(new Pose(-36,8  ,Math.PI),true))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            while((!driveToPointAsync(new Pose(60,8,Math.PI),false))&&opModeIsActive()){
                asyncPositionCorrector();
            }
            while((!driveToPointAsync(new Pose(60,12,Math.PI/2),true))&&opModeIsActive()){
                asyncPositionCorrector();
                if(armAsync(0.6)){
                    stringAsync(VOLTSSTRINGUP);
                }
           }
            pixelPlaceAuto(Location.LEFT,true);
            sleep(400);
            returnArm();
        }
        while((!driveToPointAsync(new Pose(-58.5,11,Math.PI/2),true))&&opModeIsActive()){
            asyncPositionCorrector();
        }
        turnRobot(Math.PI/8);
        turnRobot(Math.PI/2);
        Forward();
        intakeMotor.setPower(.6);
        sleep(200);
        motorsStop();
        while((!driveToPointAsync(new Pose(58,11,-Math.PI/2),true))&&opModeIsActive()){
            asyncPositionCorrector();
        }
        intakeMotor.setPower(0);
    }

    }


