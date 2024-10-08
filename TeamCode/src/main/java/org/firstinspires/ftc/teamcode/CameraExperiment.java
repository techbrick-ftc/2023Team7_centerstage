package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Camera Experiment", group = "Utility")
public class CameraExperiment extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {

        // position 1 is left, position 2 is center, position 3 is right

        FindPixel colorDetector = new FindPixel(60, 15, true, hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            packet.put("record", colorDetector.record);
            packet.put("record2", colorDetector.record2);
            packet.put("highest", colorDetector.highest);
            packet.put("highest2", colorDetector.highest2);
            dashboard.sendTelemetryPacket(packet);

        }
    }
}
