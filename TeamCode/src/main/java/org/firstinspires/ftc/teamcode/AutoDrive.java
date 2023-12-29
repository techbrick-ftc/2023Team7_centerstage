package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "ticks")
public class AutoDrive extends StarterAuto {


    @Override
    public void runOpMode() {

        //initialize(new Pose(0,0,0));
        initTfod();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
        waitForStart();
        while (opModeIsActive()) {
            //asyncPositionCorrector();

            TelemetryPacket packet = new TelemetryPacket();
            //asyncPositionCorrector();
//                packet.put("armpot",armPot.getVoltage());
//                packet.put("stringpot",stringPot.getVoltage());
//                packet.put("lifter",lifterMotor.getCurrentPosition());
//                packet.put("Field Pose", fieldPose);
            telemetryTfod();
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.

            // Share the CPU.
            sleep(20);
            dashboard.sendTelemetryPacket(packet);
        }
    }}

