package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Camera Experiment", group = "Utility")
public class CameraExperiment extends LinearOpMode {
    OpenCvCamera camera;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        int pos1X = 124;
        int pos1Y = 310;
        int pos2X = 392;
        int pos2Y = 262;
        int pos3X = 660;
        int pos3Y = 278;
        Point[] points = {new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y)};


        ColorDetector colorDetector = new ColorDetector(points, 10, 10, false, hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            //packet.put("x", 3.7);
            packet.put("location", colorDetector.location);
            dashboard.sendTelemetryPacket(packet);


        }
    }
}
