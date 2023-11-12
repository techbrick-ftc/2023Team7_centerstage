package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "ticks")
public class AutoDrive extends StarterAuto {


    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(0,0,0));
        waitForStart();
        zeroAngle = getCurrentPose().angle;
        long currentTime = System.nanoTime();
        long previousTime = System.nanoTime();
        boolean done = false;

        //while(opModeIsActive() && !done) {
        asyncPositionCorrector();
        driveToPoint(new Pose(0,50,0),true);

        packet.put("Field Pose",fieldPose);
//            packet.put("velocity Pose",velocityPose.angle);
//            dashboard.sendTelemetryPacket(packet);
        //}
        while(opModeIsActive()){
            asyncPositionCorrector();
            //done = driveToPoint(new Pose(96,48,0),true);
        }
        //turnRobot(Math.PI/2);
    }
    }

