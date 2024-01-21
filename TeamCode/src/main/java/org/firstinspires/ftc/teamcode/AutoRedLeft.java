package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoRedLeft")
public class AutoRedLeft extends StarterAuto {
    private double secondCycleY = 0;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(-36,-64,0));
        //zeroAngle = getCurrentPose().angle;
        ColorDetector colorDetector = new ColorDetector(10, 10, true, hardwareMap);
        FindPixel findPixel= new FindPixel(60, 10, true, hardwareMap);
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
                while((!driveToPointAsync(new Pose(-36,-39 ,0),false))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-53,-39,0),false))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-53,-11,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-5,-11,Math.PI/2),false))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
            while((!driveToPointAsync(new Pose(58,-14.5,Math.PI/2),true))&&opModeIsActive()){
                asyncPositionCorrector();
//                if(armAsync(ARMROTATE0POSITION)){
//                    stringAsync(STRINGVOLTTOP);
//                }
            }
            getReadyToPlace();
            pixelPlaceAuto(Location.CENTER,false);
            sleep(400);
            returnArm();

                // rotate and then move or spline under gate past E towards center of backdrop
            }
            else if (colorDetector.location == Location.LEFT) {
                while((!driveToPointAsync(new Pose(-48,-40,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                releasePixel();
                while((!driveToPointAsync(new Pose(-60,-40,0),false))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-60,-10,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(58,-9.5,Math.PI/2),true))&&opModeIsActive()){
                    asyncPositionCorrector();
//                    if(armAsync(ARMROTATE0POSITION)){
//                        stringAsync(STRINGVOLTTOP);
//                    }
                }
            getReadyToPlace();
            pixelPlaceAuto(Location.LEFT,false);
            sleep(400);
            returnArm();

            }
            else{ //right, (-27, -45)
                while((!driveToPointAsync(new Pose(-36,-40,0),false))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-22.5,-40,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                releasePixel();
                while((!driveToPointAsync(new Pose(-36,-40,0),false))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(-36,-10,0),true))&&opModeIsActive()){
                    asyncPositionCorrector();
                }
                while((!driveToPointAsync(new Pose(58,-14.5,Math.PI/2),true))&&opModeIsActive()){
                    asyncPositionCorrector();
//                    if(armAsync(ARMROTATE0POSITION)){
//                        stringAsync(STRINGVOLTTOP);
//                    }
                }
            getReadyToPlace();
            pixelPlaceAuto(Location.CENTER,false);
            sleep(400);
            returnArm();
            }
//        while((!driveToPointAsync(new Pose(-56,-11,Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        intakeMotor.setPower(.4);
//        while((!driveToPointAsync(new Pose(-12,-12,-Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        secondCycleY = findPixel.record2.y;
//        intakeMotor.setPower(1);
//        while((!driveToPointAsync(new Pose(-58,findPixel.record.y,-Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        intakeMotor.setPower(0);
//        while((!driveToPointAsync(new Pose(-12,-12,-Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        intakeMotor.setPower(1);
//        while((!driveToPointAsync(new Pose(-58,secondCycleY,-Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        while((!driveToPointAsync(new Pose(58,-11,-Math.PI/2),true))&&opModeIsActive()){
//            asyncPositionCorrector();
//        }
//        intakeMotor.setPower(-1);
//        sleep(200);
//        intakeMotor.setPower(0);
        }

    }


