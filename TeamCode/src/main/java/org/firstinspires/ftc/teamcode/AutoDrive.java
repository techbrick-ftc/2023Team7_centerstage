package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "ticks")
public class AutoDrive extends StarterAuto {


    @Override
    public void runOpMode() {

        initialize(new Pose(-36,64,0));
        long currentTime = System.nanoTime();
        long previousTime = System.nanoTime();
        boolean done = false;


        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
//        returnArm();
//        sleep(1000);
//        pixelPlaceAuto(Location.CENTER,true);
//        //stringMotor.setPower(.2);
//        while(!driveToPointAsync(new Pose(-36,64,Math.PI),true)){
//            asyncPositionCorrector();
//        }
        while (opModeIsActive()) {
            //ColorDetector colorDetector = new ColorDetector(points, 10, 10, false, hardwareMap);
            asyncPositionCorrector();
            //asyncPositionCorrector();
            TelemetryPacket packet = new TelemetryPacket();
//            packet.put("String", stringPot.getVoltage());
//            packet.put("Arm", armPot.getVoltage());
            //while (driveToPointAsync(new Pose(-24, 96, 0), true)) {
              // asyncPositionCorrector();
            //}
            //armFlipper.setPosition(-1); puts flipper up
//          sleep(3000);
//        intakeMotor.setPower(0);
           // turnRobot(Math.PI/2);
//            sleep(1500);
//            returnArm();
            //turnRobot(Math.PI/2);
            asyncPositionCorrector();
                packet.put("armpot",armPot.getVoltage());
                packet.put("stringpot",stringPot.getVoltage());
                packet.put("Field Pose", fieldPose);
                //packet.put("location?", colorDetector.location);
                dashboard.sendTelemetryPacket(packet);
                //done = driveToPoint(new Pose(96,48,0),true);
            //}
            //turnRobot(Math.PI/2);
//            boolean armDone = false;
//            while (!armDone) {
//                armDone = armAsync(ARMROTATE0POSITION, false, 0.5);
//            }
        }
    }}

