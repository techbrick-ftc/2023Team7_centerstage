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
        initialize(new Pose(0,0,0));
        double zeroAngle = 0;
        boolean speedMod = true;
        boolean fieldCentric = true;
        double rotX = 0;
        double rotY = 0;
        double rx = 0;
        double wristPosition = 0.92;
        boolean grabberOpen = false;
        Gamepad previousGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad cur2 = new Gamepad();
        Gamepad cur1 = new Gamepad();

        waitForStart();

        zeroAngle = getCurrentPose().angle;
        while (opModeIsActive()) {
            try {
                cur2.copy(gamepad2);
                cur1.copy(gamepad1);
            } catch (RuntimeException e) {

            }
            if (cur1.right_bumper && !previousGamepad1.right_bumper) {
                speedMod = true;
            } else if (cur1.left_bumper && !previousGamepad1.left_bumper) {
                speedMod = false;
            }
            telemetry.update();
            double driveYleftStick = -gamepad1.left_stick_y; // Remember, this is reversed!
            double driveXleftStick = gamepad1.left_stick_x; // Counteract imperfect strafing
            double armYleftStick = -gamepad1.left_stick_y; // Remember, this is reversed!
            double armXleftStick = gamepad1.left_stick_x; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;
            double armRightTrigger = gamepad2.right_trigger;
            double armLeftTrigger = gamepad2.left_trigger;
            boolean driveA = cur1.a;
            boolean driveB = cur1.b;
            boolean driveX = cur1.x;
            boolean driveY = cur1.y;
            boolean armA = cur2.a;
            boolean armB = cur2.b;
            boolean armX = cur2.x;
            boolean armY = cur2.y;
            boolean armDpadUp = cur2.dpad_up;
            boolean armDpadDown = cur2.dpad_down;
            boolean previousDriveA = previousGamepad1.a;
            boolean previousDriveB = previousGamepad1.b;
            boolean previousDriveX = previousGamepad1.x;
            boolean previousDriveY = previousGamepad1.y;
            boolean previousArmA = previousGamepad2.a;
            boolean previousArmB = previousGamepad2.b;
            boolean previousArmX = previousGamepad2.x;
            boolean previousArmY = previousGamepad2.y;
            boolean previousDpadUp = previousGamepad2.dpad_up;
            boolean previousDpadDown = previousGamepad2.dpad_down;
            Pose current = getCurrentPose();
            if (!speedMod) {
                driveYleftStick = Range.clip(-gamepad1.left_stick_y, -0.4, 0.4);
                driveXleftStick = Range.clip(gamepad1.left_stick_x, -0.4, 0.4);
                rx = Range.clip(gamepad1.right_stick_x, -0.25, 0.25);
            } else {
                driveYleftStick = Range.clip(-gamepad1.left_stick_y, -0.95, 0.95);
                rx = Range.clip(gamepad1.right_stick_x, -0.75, 0.75);
            }
            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -(current.angle );
            if (driveY) {
                zeroAngle = current.angle;
            }
            //Ask if we still want this or change be to be something useful
            if (driveB && !previousArmB) {
                fieldCentric = !fieldCentric;
            }
            if (fieldCentric) {
                rotX = driveXleftStick * Math.cos(botHeading) - driveYleftStick * Math.sin(botHeading);
                rotY = driveXleftStick * Math.sin(botHeading) + driveYleftStick * Math.cos(botHeading);
            } else {
                rotX = driveXleftStick;
                rotY = driveYleftStick;
            }
            //Moves the arm back and forth
            if(!(armXleftStick<0.05 && armXleftStick>-0.05)){
                armMove(armXleftStick);
            }
            //Moves the strings out and in
            if((armRightTrigger<0.05)||(armLeftTrigger<0.05)){
                stringMove(armRightTrigger,armLeftTrigger);
            }
            //Moves the servo
            if((armDpadUp || previousDpadUp || armDpadDown || previousDpadDown)){
                lift(armDpadUp,previousDpadUp,armDpadDown,previousDpadDown);
            }
            if(armA && !previousArmA){
                if(currentPosition == 2){
                    setFinger(servoPositions[1]);
                    currentPosition = 1;
                    increasingPosition = "decreasing";
                }
                else if(currentPosition==1){
                    if(increasingPosition.equals("increasing")){
                        setFinger(servoPositions[2]);
                        currentPosition = 2;
                    }
                    else{
                        setFinger(servoPositions[0]);
                        currentPosition = 0;
                    }
                }
                else{
                    setFinger(servoPositions[1]);
                    currentPosition = 1;
                    increasingPosition="increasing";
                }
            }



            packet.put("rotatex", rotX);
            packet.put("rotatey", rotY);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower = (rotY + rotX - rx) / denominator;
            double backLeftPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX + rx) / denominator;
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
            packet.put("imu x ", Math.toDegrees(current.angle));
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