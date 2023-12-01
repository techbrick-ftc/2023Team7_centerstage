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
    OpenCvCamera camera;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        int pos1X = 30;
        int pos1Y = 390;
        int pos2X = 440;
        int pos2Y = 305;
        int pos3X = 855;
        int pos3Y = 360;
        Point[] points = {new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y)};
// position 1 is left, position 2 is center, position 3 is right

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
