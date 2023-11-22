package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "ticks")
public class AutoDrive extends StarterAuto {


    @Override
    public void runOpMode() {

        initialize(new Pose(0,0,0));
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
        Point[] points = {new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y)};


        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        // detect the colour (positions are estimates)
        //robot is about 16 inches long

        //while(opModeIsActive() && !done) {

        lifterMotor.setPower(.2);

//            packet.put("velocity Pose",velocityPose.angle);

        //}
        while(opModeIsActive()){
            asyncPositionCorrector();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("String", stringPot.getVoltage());
            packet.put("Arm", armPot.getVoltage());
            packet.put("Field Pose",fieldPose);
            dashboard.sendTelemetryPacket(packet);
            //done = driveToPoint(new Pose(96,48,0),true);
        }
        //turnRobot(Math.PI/2);
    }
    }

