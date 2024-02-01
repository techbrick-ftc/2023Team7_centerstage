package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "tick")
public class AutoDrive extends StarterAuto {

    @Override
    public void runOpMode() {

        // initialize(new Pose(0,0,0));
        // FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        initialize(new Pose(0, 0, 0));

        // asyncPositionCorrector();
        // packet.put("record",colorDetector.record);
        // packet.put("record2",colorDetector.record2);
        //
        // packet.fieldOverlay()
        // .drawImage("robot.jpg", fieldPose.x-8, fieldPose.y-8, 16, 16);
        dashboard.sendTelemetryPacket(packet);

        // FindPixel colorDetector = new FindPixel(60, 15, true, hardwareMap);
        while (opModeInInit()) {
            packet.put("string",
                    ((stringPot.getVoltage() < .2) ? (stringPot.getVoltage() + stringPot.getMaxVoltage()) : (stringPot.getVoltage())));
            packet.put("stringreal", (stringPot.getVoltage()));
            dashboard.sendTelemetryPacket(packet);
        }
        waitForStart();
        // getReadyToPlace();

        // intakeMotor.setPower(.5);
        // finger.setPosition(0.5); maks it hold stuff
        // armFlipper.setPosition(.3);

        while (opModeIsActive()) {
            ;
            finger.setPosition(1);
            sleep(3000);
            finger.setPosition(.5);
            sleep(3000);
            finger.setPosition(0);
            sleep(3000);
            dashboard.sendTelemetryPacket(packet);
            // armFlipper.setPosition(.3);

            // armMotor.setPower(.4);
            // while((armAsync(2.4))&&(opModeIsActive())){
            // }

            // while((stringAsync(STRINGVOLTDOWN))&&(opModeIsActive())){
            // }
            // packet.put("record",colorDetector.record);
            // packet.put("record2",colorDetector.record2);
            //
            // packet.fieldOverlay()
            // .drawImage("robot.jpg", fieldPose.x-8, fieldPose.y-8, 16, 16);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
