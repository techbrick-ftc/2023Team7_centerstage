package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.lang.Math;

/*
Configurations:
Expansion Hub:
 I2C Port 0: colorBL     aka color1
 I2C Port 1: colorBR
 I2C Port 2: colorFR
 I2C Port 3: colorFL

 Servo Port 0:wristServo



 Motor Port 1: armMotor
 Motor Port 0: stringMotor

 Control Hub:
  Motor Port 0: frontRight
  Motor Port 1: frontLeft
  Motor Port 2: backLeft
  Motor Port 3: backRight

  I2C Port 0: imu

  Digital 1: armuptouch

  Analog 0 : stringpot

  Analog 2: armpot

  Servo Port 5:grabbaServo


 */
class Pose {
    Pose(double x, double y, double a) {
        this.x = x;
        this.y = y;
        angle = a;
    }

    public double x;
    public double y;

    public double angle;

    @NonNull
    public String toString() {
        return "Pose(X = " + x + ", Y = " + y + ", Angle in Radians = " + angle + ", Angle in Degrees = " + Math.toDegrees(angle) + ")";
    }
}

public class StarterAuto extends LinearOpMode {

    double lastTime;
    double startDecel = 30;

    double stopDecel = 0;

    static Pose fieldPose = new Pose(0, 0, 0);
    Pose velocityPose = new Pose(0, 0, 0);
    double zeroAngle = 0;
    double ticksPerRadian = 28.58 * (360 / (Math.PI * 2)); // found 28.58 by using testangular method to find ratio of ticks to radian
    public double inPerTick = 20 / 6786.0;
    static final double FEET_PER_METER = 3.28084;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    public final FtcDashboard dashboard = FtcDashboard.getInstance();
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public IMU imu;
    public DcMotorEx deadPerp;

    public DcMotorEx deadLeft;

    public DcMotorEx deadRight;
    public DcMotor armMotor;
    public DcMotor stringMotor;
    public ColorSensor colorFR;
    public ColorSensor colorFL;
    public ColorSensor colorBR;
    public ColorSensor colorBL;
    public TouchSensor armuptouch;

//    public AnalogInput armpot;

    double previousAngle = 0;

    double spinCounter = 0;
    int numFramesWithoutDetection = 0;
    int[] arrayDetections = new int[64];
    int detectionIndex = 0;
    double maxVelocity = 70.0;
    //Max forward velocity is 80 in/sec
    //Max strafe velocity is 70 in/sec

    double minimumPower = .3;
    private Pose lastPose;


    void asyncPositionCorrector() {
        TelemetryPacket packet = new TelemetryPacket();
        if (lastPose == null) {
            lastPose = getCurrentPose();
            return;
        }
        Pose current = getCurrentPose();
        double currentTime = System.currentTimeMillis() / 1000.0;
        double achange = current.angle - lastPose.angle;
        double xchange = current.x - lastPose.x;
        double ychange = current.y - lastPose.y;
        if (achange > Math.PI) {
            achange -= Math.PI * 2;
        } else if (achange < -Math.PI) {
            achange += Math.PI * 2;

        }
        double correctedx = xchange - achange * ticksPerRadian * inPerTick;
        double rotA = current.angle / 2 + lastPose.angle / 2;
        double rotX = correctedx * Math.cos(rotA) - ychange * Math.sin(rotA);
        double rotY = correctedx * Math.sin(rotA) + ychange * Math.cos(rotA);
        lastPose = current;
        fieldPose.x += rotX;
        fieldPose.y += rotY;
        fieldPose.angle = current.angle;
        velocityPose.x = rotX / ((currentTime - lastTime));
        velocityPose.y = rotY / ((currentTime - lastTime));
        velocityPose.angle = achange / (currentTime - lastTime);
        lastTime = currentTime;
        packet.put("Field Pose", fieldPose);
        dashboard.sendTelemetryPacket(packet);
    }

    Pose getCurrentPose() {
        double y = (deadLeft.getCurrentPosition() * inPerTick + deadRight.getCurrentPosition() * inPerTick) / 2;
        return new Pose(deadPerp.getCurrentPosition() * inPerTick, y, (imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle - zeroAngle));
    }

    public void setPower(DcMotor motor, double targetPower, String name) {
        TelemetryPacket packet = new TelemetryPacket();
        double currentPower = motor.getPower();
        if (Math.abs(targetPower) - .1 > Math.abs(currentPower)) {

            double powerChange = 0.0125 * Math.signum(targetPower); // Adjust power based on the sign of targetPower

            double newPower = currentPower + powerChange;

            // Make sure newPower stays within the valid range of -1.0 to 1.0
            newPower = Math.max(-1.0, Math.min(1.0, newPower));


            motor.setPower(newPower);
            packet.put(name, newPower);
        } else {
            if (Math.abs(targetPower) > 0 && Math.abs(targetPower) < 0.2) {
                targetPower = Math.signum(targetPower) * .2;
            }
            motor.setPower(targetPower);
            packet.put(name, targetPower);
        }
        dashboard.sendTelemetryPacket(packet);
    }

    public void testAngular() {
        TelemetryPacket packet = new TelemetryPacket();
        frontRight.setPower(-1);  // front
        frontLeft.setPower(1);    // left
        backRight.setPower(-1);    // right
        backLeft.setPower(1);
        while (opModeIsActive()) {

            Pose current = new Pose(deadPerp.getCurrentPosition(), 0, positiveWrap((getCurrentPose().angle)));
            current.angle = Math.toDegrees(current.angle);
            packet.put("x", current.x);
            packet.put("angkle", current.angle);
            packet.put("previousangle", previousAngle);
            if (current.angle > previousAngle) {
                spinCounter += 1;
            }
            packet.put("spincointer", spinCounter);
            packet.put("ratio", (current.x / (-spinCounter * 360 + current.angle)));
            previousAngle = (current.angle);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public boolean driveToPointAsync(Pose target, boolean slowDown) {
        TelemetryPacket packet = new TelemetryPacket();
        Pose cur = fieldPose; // our current poe
        Pose diff = new Pose(target.x - cur.x, target.y - cur.y, wrap((target.angle) - (cur.angle))); // difference in points
        packet.put("Diff", diff);
        // uses angles to find rotated X and Y
        double rotX = diff.x * Math.cos(cur.angle) - diff.y * Math.sin(cur.angle);
        double rotY = diff.x * Math.sin(cur.angle) + diff.y * Math.cos(cur.angle);
        double denom = Math.max(Math.abs(rotX), Math.abs(rotY));
        if (denom != 0) {
            rotX = rotX / denom;
            rotY = rotY / denom;
        }
        double angleFactor = diff.angle;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(angleFactor), 1);
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double multiplier = deceleration(slowDown, diff.x, diff.y, diff.angle);
        packet.put("Multiplier", multiplier);
        double frontLeftPower = ((rotY + rotX - angleFactor) / denominator) * multiplier;
        double backLeftPower = ((rotY - rotX - angleFactor) / denominator) * multiplier;
        double frontRightPower = ((rotY - rotX + angleFactor) / denominator) * multiplier;
        double backRightPower = ((rotY + rotX + angleFactor) / denominator) * multiplier;

//        packet.put("frontleftPower",frontLeftPower);
//        packet.put("fronrightPower",frontRightPower);
//        packet.put("backrightpower",backRightPower);
//        packet.put("backleftPower",backLeftPower);
        setPower(frontRight, frontRightPower, "frontRight");
        setPower(frontLeft, frontLeftPower, "frontLeft");
        setPower(backRight, backRightPower, "backRight");
        setPower(backLeft, backLeftPower, "backLeft");


        dashboard.sendTelemetryPacket(packet);
        if (multiplier == 0) {
            return true;
        }
        return false;
    }
        void driveToPoint (Pose target,boolean slowDown){
            boolean done = false;
            while (!done && opModeIsActive()) {
                done = driveToPointAsync(target, slowDown);
            }
        }
        protected void motorsStop () {
            backRight.setPower(0);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        protected double deceleration ( boolean slow, double rotX, double rotY, double angleDiff){
            boolean slowDown = slow;
            double angleConstant = .1 / (10 * (Math.PI * 2) / 360);
            double d = Math.sqrt(((rotX * rotX) + (rotY * rotY)));// + Math.abs(angleDiff * angleConstant); Accounting for angle with distance
            double powerLinear = 0;


            if ((d <= stopDecel)) {
                return 0;
            }
            if (slowDown) {
                if (d < startDecel) {
                    powerLinear = (((1 - minimumPower) / (startDecel - stopDecel)) * (d - stopDecel) + minimumPower);
                    //will want to change velocityRatio to a real algorithm
                    double velocityRatio = (Math.sqrt((velocityPose.x * velocityPose.x) + (velocityPose.y * velocityPose.y)) / maxVelocity);
                    double finalPower = powerLinear - velocityRatio;
                    return finalPower;
                }
//           else if (targetspeed - speed > 0.02) {
//                //Motors would get faster
//                return 1;
                //  }
                else {
                    //Speed is normal
                    return 1;
                }
            } else if (d < .5) {
                return 0;

            } else if (d < 6) {
                return .5;
            }

            return 1;
        }


        private double wrap ( double theta){
            double newTheta = theta;
            while (Math.abs(newTheta) > Math.PI) {
                if (newTheta < -Math.PI) {
                    newTheta += Math.PI * 2;
                } else {
                    newTheta -= Math.PI * 2;
                }
            }
            return newTheta;
        }

        private double positiveWrap ( double theta){
            double newTheta = theta;
            while (newTheta > Math.PI * 2) {
                newTheta -= Math.PI * 2;
            }
            while (newTheta < 0) {
                newTheta += Math.PI * 2;
            }
            return newTheta;
        }


        protected void initialize (Pose inputPose){
            //Hardware map does not line up with names, may want to change this.
            frontLeft = hardwareMap.get(DcMotor.class, "backRight");
            backLeft = hardwareMap.get(DcMotor.class, "frontRight");
            frontRight = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "frontLeft");


            deadLeft = hardwareMap.get(DcMotorEx.class, "par1");
            deadRight = hardwareMap.get(DcMotorEx.class, "par0");
            deadPerp = hardwareMap.get(DcMotorEx.class, "perp");
            deadLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            deadRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            deadPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            deadPerp.setDirection(DcMotorSimple.Direction.REVERSE);
            deadLeft.setDirection(DcMotorSimple.Direction.REVERSE);


            colorFR = hardwareMap.colorSensor.get("colorFR");
            colorFL = hardwareMap.colorSensor.get("colorFL");
            colorBR = hardwareMap.colorSensor.get("colorBR");
            colorBL = hardwareMap.colorSensor.get("colorBL");


            imu = hardwareMap.get(IMU.class, "imu");
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);

            telemetry.update();

            fieldPose = inputPose;
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            zeroAngle = (imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle - zeroAngle) - inputPose.angle;
        }


        void turnRobot ( double angle){
            double difAngle = wrap(positiveWrap(angle) - positiveWrap(fieldPose.angle));
            double directionalSpeed = Math.signum(difAngle) * 0.5;
            while (opModeIsActive() && !((difAngle) < Math.toRadians(2))) {
                asyncPositionCorrector();
                if (Math.abs(difAngle) < 0.05 * Math.PI) {
                    directionalSpeed *= 0.3;
                } else if (Math.abs(difAngle) < 0.1 * Math.PI) {
                    directionalSpeed *= 0.5;
                }
                difAngle = wrap(positiveWrap(angle) - positiveWrap(fieldPose.angle));


                setPower(backLeft, directionalSpeed, "backLeft");
                setPower(backRight, -directionalSpeed, "backRight");
                setPower(frontLeft, directionalSpeed, "frontLeft");
                setPower(frontRight, -directionalSpeed, "frontRight");

            }

            setPower(backLeft, 0, "backLeft");
            setPower(backRight, 0, "backRight");
            setPower(frontLeft, 0, "frontLeft");
            setPower(frontRight, 0, "frontRight");
        }

        protected void imuAngle () {
            telemetry.addData("IMU Angle", getCurrentPose().angle);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("IMU Angle", getCurrentPose().angle);
            dashboard.sendTelemetryPacket(packet);
        }

        protected void stringMove ( double rightTrigger, double leftTrigger){
            if (rightTrigger > 0) {
//            if (stringpot.getVoltage() <= VOLTSSTRINGUP) {
//                stringMotor.setPower(0);
//                stringPotLastVal = stringpot.getVoltage();
//            } else {
                stringMotor.setPower(-rightTrigger);
//                stringPotLastVal = stringpot.getVoltage();

            } else if (leftTrigger > 0) {
//            if (stringpot.getVoltage() >= VOLTSSTRINGDOWN) {
//                stringMotor.setPower(0);
//                stringPotLastVal = stringpot.getVoltage();
//
//            } else {
                stringMotor.setPower(leftTrigger);
//                stringPotLastVal = stringpot.getVoltage();
            }
        }
        protected void armMove ( double leftStickX){
            armMotor.setPower(leftStickX);
        }
//comment #2
        @Override
        public void runOpMode () {

        }
    }
