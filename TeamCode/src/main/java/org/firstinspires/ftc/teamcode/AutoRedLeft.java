package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "AutoRedLeft")
public class AutoRedLeft extends StarterAuto {


    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(-36,-64,0));
        waitForStart();
        zeroAngle = getCurrentPose().angle;
        long currentTime = System.nanoTime();
        long previousTime = System.nanoTime();
        boolean done = false;
        int pos1X = 55;
        int pos1Y = 270;
        int pos2X = 440;
        int pos2Y = 245;
        int pos3X = 765;
        int pos3Y = 270;
        Point[] points = {new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y)};


        ColorDetector colorDetector = new ColorDetector(points, 10, 10, true, hardwareMap);
        // We wan to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        // detect the colour (positions are estimates)
        //robot is about 16 inches long
        if (colorDetector.location == Location.CENTER) {


                while(!driveToPointAsync(new Pose(-36,-38,0),true)){
                    asyncPositionCorrector();

                }
                releasePixel();
                while(!driveToPointAsync(new Pose(-36,-39 ,0),true)){
                    asyncPositionCorrector();
                }
                while(!driveToPointAsync(new Pose(-56,-45,0),true)){
                    asyncPositionCorrector();
                }
                while(!driveToPointAsync(new Pose(-56,-11,0),true)){
                    asyncPositionCorrector();
                }
                while(!driveToPointAsync(new Pose(60,-11,0),true)){
                    asyncPositionCorrector();
                }

                // rotate and then move or spline under gate past E towards center of backdrop
            }
            else if (colorDetector.location == Location.LEFT) {
                while(!driveToPointAsync(new Pose(-48,-40,0),true)){
                    asyncPositionCorrector();
                }
                releasePixel();
                while(!driveToPointAsync(new Pose(-60,-40,0),true)){
                    asyncPositionCorrector();
                }
                while(!driveToPointAsync(new Pose(-60,-12,0),true)){
                    asyncPositionCorrector();
                }
                while(!driveToPointAsync(new Pose(60,-12,0),true)){
                    asyncPositionCorrector();
                }


            }
            else{ //right, (-27, -45)
                while(!driveToPointAsync(new Pose(-36,-40,0),true)){
                    asyncPositionCorrector();
                }
                while(!driveToPointAsync(new Pose(-24,-40,0),true)){
                    asyncPositionCorrector();
                }
                releasePixel();
                while(!driveToPointAsync(new Pose(-36,-40,0),true)){
                    asyncPositionCorrector();
                }
                while(!driveToPointAsync(new Pose(-36,-12,0),true)){
                    asyncPositionCorrector();
                }
                while(!driveToPointAsync(new Pose(60,-12,0),true)){
                    asyncPositionCorrector();
                }
            }
        }
        //turnRobot(Math.PI/2);
    }


