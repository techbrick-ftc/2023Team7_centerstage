package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
class Pose{
    Pose(double x, double y, double a)
    {
        this.x = x;
        this.y = y;
        angle = a;
    }
    public double x;
    public double y;

    public double angle;
    public String toString()
    {
        return "Pose(X = " + x + ", Y = " + y + ", Angle in Radians = " + angle + ", Angle in Degrees = " + Math.toDegrees(angle) + ")";
    }
}

public class StarterAuto extends LinearOpMode {
    double zeroAngle = 0;
    public double inPerTick = 20/6786.0;
    public double lateralInPerTick =20/6786.0;
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
    public BNO055IMU imu;
    public DcMotorEx deadPerp;

    public DcMotorEx deadLeft;

    public DcMotorEx deadRight;
    public ColorSensor colorFR;
    public ColorSensor colorFL;
    public ColorSensor colorBR;
    public ColorSensor colorBL;
    public TouchSensor armuptouch;
//    public AnalogInput stringpot;
//    public AnalogInput armpot;
   // OpenCvCamera camera;
    double fx = 1481.603;
    double fy = 1527.539;
    double cx = 550.003;
    double cy = 90.751;
    // UNITS ARE METERS
    double tagsize = 0.045;
    int numFramesWithoutDetection = 0;
    int[] arrayDetections = new int[64];
    int detectionIndex = 0;

    public void insertDetection(int value) {
        arrayDetections[detectionIndex] = value;
        detectionIndex++;
        if (detectionIndex == arrayDetections.length) {
            detectionIndex = 0;
        }
    }

    void drivingCorrectionStraight(double startAngle2, double power) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double difference = imu.getAngularOrientation().firstAngle - startAngle2;
        difference /= Math.PI * 2;
        difference *= power;
        backRight.setPower(power - difference);
        backLeft.setPower(power + difference);
        frontLeft.setPower(power + difference);
        frontRight.setPower(power - difference);
    }
    Pose getCurrentPose(){
        double y = (deadLeft.getCurrentPosition()*inPerTick+deadRight.getCurrentPosition()*inPerTick)/2;
        Pose current = new Pose(deadPerp.getCurrentPosition()*inPerTick,y,(imu.getAngularOrientation().firstAngle - zeroAngle));
        return current;
    }

    public
    void driveToPoint(Pose target){
        TelemetryPacket packet = new TelemetryPacket();
        Pose cur = getCurrentPose();
        Pose coord = new Pose(target.x-cur.x,target.y-cur.y,wrap(positiveWrap(target.angle)-positiveWrap(cur.angle)));
        packet.put("Dif",coord);
        double rotX = coord.x * Math.cos(cur.angle) - coord.y * Math.sin(cur.angle);
        double rotY = coord.x * Math.sin(cur.angle) + coord.y * Math.cos(cur.angle);
        double denom = Math.max(Math.abs(rotX),Math.abs(rotY));
        if(denom != 0){
            rotX = rotX/denom;
            rotY = rotY/denom;
        }

        packet.put("Current",cur);
        packet.put("rotx",rotX);
        packet.put("roty",rotY);
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(coord.angle), 1);


            double frontLeftPower = ((rotY + rotX - coord.angle) / denominator)*.4;
            double backLeftPower = ((rotY - rotX - coord.angle) / denominator)*.4;
            double frontRightPower = ((rotY - rotX + coord.angle ) / denominator)*.4;
            double backRightPower = ((rotY + rotX + coord.angle) / denominator)*.4;

//        double frontLeftPower = (rotY + rotX - endingAngle);
//        double backLeftPower = (rotY - rotX - endingAngle);
//        double frontRightPower = (rotY - rotX + endingAngle);
//        double backRightPower = (rotY + rotX + endingAngle);
        packet.put("frontleftPower",frontLeftPower);
        packet.put("fronrightPower",frontRightPower);
        packet.put("backrightpower",backRightPower);
        packet.put("backleftPower",backLeftPower);
        frontRight.setPower(frontRightPower);  // front
        frontLeft.setPower(frontLeftPower);    // left
        backRight.setPower(backRightPower);    // right
        backLeft.setPower(backLeftPower);      // back
        dashboard.sendTelemetryPacket(packet);
    }

    void drivingCorrectionLeft(double startAngle, double power) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double difference = imu.getAngularOrientation().firstAngle - startAngle;
        difference /= Math.PI * 2;
        difference *= power;
        frontRight.setPower(power - difference);
        frontLeft.setPower(-power + difference);
        backLeft.setPower(power + difference);
        backRight.setPower(-power - difference);
    }

    protected void motorsStop() {
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }




    private double wrap(double theta) {
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
    private double positiveWrap(double theta)
    {
        double newTheta = theta;
        while(newTheta>Math.PI*2)
        {
            newTheta -=Math.PI*2;
        }
        while(newTheta<0)
        {
            newTheta += Math.PI*2;
        }
        return newTheta;
    }

    boolean shouldStopTurning(double targetAngle) {
        double currentAngle = imu.getAngularOrientation().firstAngle;
        return Math.abs(currentAngle - targetAngle) < .005 * Math.PI;
    }

    protected void initialize() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        deadLeft = hardwareMap.get(DcMotorEx.class,"par1");
        deadRight = hardwareMap.get(DcMotorEx.class,"par0");
        deadPerp = hardwareMap.get(DcMotorEx.class,"perp");
        deadLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        colorFR = hardwareMap.colorSensor.get("colorFR");
        colorFL = hardwareMap.colorSensor.get("colorFL");
        colorBR = hardwareMap.colorSensor.get("colorBR");
        colorBL = hardwareMap.colorSensor.get("colorBL");




        imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
    }







    void turnRobot(double angle, boolean clockwise) {
        double directionalSpeed = clockwise ? 1 : -1;
        double targetAngle = wrap(imu.getAngularOrientation().firstAngle + angle);
        while (opModeIsActive() && !shouldStopTurning(targetAngle)) {

            telemetry.addData("angle", imu.getAngularOrientation().firstAngle);
            telemetry.update();

            if (Math.abs(imu.getAngularOrientation().firstAngle - targetAngle) < 0.05 * Math.PI) {
                directionalSpeed *= 0.3;
            } else if (Math.abs(imu.getAngularOrientation().firstAngle - targetAngle) < 0.1 * Math.PI) {
                directionalSpeed *= 0.5;
            }

            backLeft.setPower(directionalSpeed);
            frontRight.setPower(-directionalSpeed);
            backRight.setPower(-directionalSpeed);
            frontLeft.setPower(directionalSpeed);

        }

        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
    }

    protected void imuAngle() {
        telemetry.addData("IMU Angle", imu.getAngularOrientation().firstAngle);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("IMU Angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void runOpMode() {

    }
}