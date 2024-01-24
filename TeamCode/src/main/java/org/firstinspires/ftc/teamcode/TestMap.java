package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestMap")
public class TestMap extends StarterAuto {

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            //packet.fieldOverlay()
                   // .drawImage("robot.jpg", 24, 24, 16, 16);
            packet.fieldOverlay()
                    .setFill("blue")
                    .fillRect(-20, -20, 40, 40);
            dashboard.sendTelemetryPacket(packet);
        }
        // turnRobot(Math.PI/2);
    }
}