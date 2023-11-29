package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous(name = "ticks")
public class AutoDrive extends StarterAuto {

    @Override
    public void runOpMode() {

        initialize(new Pose(0, 0, 0));
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
        Point[] points = { new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y) };

        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        waitForStart();
        // detect the colour (positions are estimates)
        // robot is about 16 inches long

        // while(opModeIsActive() && !done) {

        // frontLeft.setPower(1);
        // frontRight.setPower(1);
        // backLeft.setPower(1);
        // backRight.setPower(1);
        // intakeMotor.setPower(-.3);
        // packet.put("velocity Pose",velocityPose.angle);

        // }
        // done = armAsync(ARMROTATE0POSITION);
        // while (opModeIsActive() && !done) {
        // done = armAsync(ARMROTATE0POSITION);
        // }
        //// done = stringAsync(1);
        //// while (opModeIsActive() && !done) {
        //// done = stringAsync(1);
        //// }
        // while(!stringAsync(VOLTSSTRINGUP+.05)){
        //
        // }
        pixelPlaceAuto(Location.CENTER);
        // stringMotor.setPower(.2);
        while (opModeIsActive()) {
            asyncPositionCorrector();
            TelemetryPacket packet = new TelemetryPacket();
            // packet.put("String", stringPot.getVoltage());
            // packet.put("Arm", armPot.getVoltage());
            // while (driveToPointAsync(new Pose(-24, 96, 0), true)) {
            // asyncPositionCorrector();
            // }
            // armFlipper.setPosition(-1); puts flipper up
            // sleep(3000);
            // intakeMotor.setPower(0);
            packet.put("armpot", armPot.getVoltage());
            packet.put("stringpot", stringPot.getVoltage());
            packet.put("Field Pose", fieldPose);
            dashboard.sendTelemetryPacket(packet);
            // done = driveToPoint(new Pose(96,48,0),true);
            // }
            // turnRobot(Math.PI/2);
            // boolean armDone = false;
            // while (!armDone) {
            // armDone = armAsync(ARMROTATE0POSITION, false, 0.5);
            // }
        }
    }
}
