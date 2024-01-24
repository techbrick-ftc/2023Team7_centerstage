package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "collectpixel")
public class FindPixelTest extends StarterAutoArmless {
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(-12, -12, Math.PI / 2));
        // zeroAngle = getCurrentPose().angle;
        FindPixel colorDetector = new FindPixel(60, 10, true, hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        while (opModeInInit()) {
            packet.put("record", colorDetector.record);
            packet.put("record2", colorDetector.record2);
            dashboard.sendTelemetryPacket(packet);
        }
        waitForStart();
        if (opModeIsActive()) {
            // change max speed -- too fast rn
            driveToPoint(new Pose(-36, -12, Math.PI / 2), true);
            driveToPoint(new Pose(-36, colorDetector.record.y, Math.PI / 2), true);
            intakeMotor.setPower(.5);
            driveToPoint(new Pose(-60, colorDetector.record.y, Math.PI / 2), true);

            driveToPoint(new Pose(-36, fieldPose.y, Math.PI / 2), true);// straight back
            driveToPoint(new Pose(-36, colorDetector.record2.y, Math.PI / 2), true); // side to match pixel
            driveToPoint(new Pose(-60, colorDetector.record2.y, Math.PI / 2), true); // forward
            intakeMotor.setPower(.5);
            driveToPoint(new Pose(60, -7, -Math.PI / 2), true);
            intakeMotor.setPower(-.5);
            sleep(500);
            intakeMotor.setPower(0);
        }
    }
}
