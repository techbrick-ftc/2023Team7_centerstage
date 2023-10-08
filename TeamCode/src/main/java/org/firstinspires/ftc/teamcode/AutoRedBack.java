package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "red autonomous")
public class AutoRedBack extends LinearOpMode  {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public BNO055IMU imu;
    public ColorSensor colorFR;
    public ColorSensor colorFL;
    public ColorSensor colorBR;
    public ColorSensor colorBL;
    OpenCvCamera camera;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    protected void initialize() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        colorFR = hardwareMap.colorSensor.get("colorFR");
        colorFL = hardwareMap.colorSensor.get("colorFL");
        colorBR = hardwareMap.colorSensor.get("colorBR");
        colorBL = hardwareMap.colorSensor.get("colorBL");
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        int pos1X = 124;
        int pos1Y = 310;
        int pos2X = 392;
        int pos2Y = 262;
        int pos3X = 660;
        int pos3Y = 278;
        Point[] points = {new Point(pos1X, pos1Y), new Point(pos2X, pos2Y), new Point(pos3X, pos3Y)};


        ColorDetector colorDetector = new ColorDetector(points, 10, 10, false, hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 0 (probably)
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(0));
        waitForStart();
        // detect the colour (positions are estimates)
        if (colorDetector.location == Location.CENTER) {
            // driving and rotating to (-48, -30)
            // rotate and then move or spline under gate past E towards center of backdrop
        }
        else if (colorDetector.location == Location.LEFT) {
            // driving to (-45, -45)
            // same as center but further, avoid D
        }
        else{ //right, (-27, -45)
            //move left towards gate, pass gate, avoid F
        }
    }

}
