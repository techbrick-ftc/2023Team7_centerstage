package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class MainTeleOp extends StarterAuto {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        initialize();
        double zeroAngle = 0;
        boolean speedMod = true;

        boolean fieldCentric = true;

        double rotX = 0;
        double rotY = 0;
        double rx = 0;


        double wristPosition = 0.92;
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        Gamepad cur2 = new Gamepad();
        Gamepad cur1 = new Gamepad();

        boolean grabberOpen = false;


        waitForStart();

        zeroAngle = getCurrentPose().angle;

        while (opModeIsActive()) {


            try {
                cur2.copy(gamepad2);
                cur1.copy(gamepad1);
            } catch (RuntimeException e) {

            }

            // right trigger extends, extending is -1  power

            if (cur1.right_bumper && !previousGamepad1.right_bumper) {
                speedMod = true;
            } else if (cur1.left_bumper && !previousGamepad1.left_bumper) {
                speedMod = false;
            }


            telemetry.update();

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;

            if (!speedMod) {
                y = Range.clip(-gamepad1.left_stick_y, -0.4, 0.4);
                x = Range.clip(gamepad1.left_stick_x, -0.4, 0.4);
                rx = Range.clip(gamepad1.right_stick_x, -0.25, 0.25);
            } else {
                y = Range.clip(-gamepad1.left_stick_y, -0.95, 0.95);
                rx = Range.clip(gamepad1.right_stick_x, -0.75, 0.75);
            }

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -(getCurrentPose().angle );

            if (gamepad1.y) {
                zeroAngle = getCurrentPose().angle;
            }
            if (cur1.b && !previousGamepad1.b) {
                fieldCentric = !fieldCentric;
            }

            if (fieldCentric) {
                rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            } else {
                rotX = x;
                rotY = y;
            }

            packet.put("rotatex", rotX);
            packet.put("rotatey", rotY);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

//            double frontLeftPower = (rotY + rotX - rx) / denominator;
//            double backLeftPower = (rotY - rotX - rx) / denominator;
//            double frontRightPower = (rotY - rotX + rx) / denominator;
//            double backRightPower = (rotY + rotX + rx) / denominator;

            double frontLeftPower = (rotY + rotX - rx);
            double backLeftPower = (rotY - rotX - rx);
            double frontRightPower = (rotY - rotX + rx);
            double backRightPower = (rotY + rotX + rx);
            double s= deadLeft.getCurrentPosition();
            double t= deadRight.getCurrentPosition();
            double d= deadPerp.getCurrentPosition();
            packet.put("deadLeft",s);
            packet.put("deadRight",t);
            packet.put("deadPerp",d);

            frontRight.setPower(frontRightPower);  // front
            frontLeft.setPower(frontLeftPower);    // left
            backRight.setPower(backRightPower);    // right
            backLeft.setPower(backLeftPower);      // back

            packet.put("zer", Math.toDegrees(zeroAngle));
            packet.put("imu x ", Math.toDegrees(getCurrentPose().angle));
            packet.put("imu y ", Math.toDegrees(imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle));
            packet.put("imu z ", Math.toDegrees(imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle));

            dashboard.sendTelemetryPacket(packet);

            try {
                previousGamepad1.copy(cur1);
                previousGamepad2.copy(cur2);
            } catch (RuntimeException e) {
            }
        }
    }
}