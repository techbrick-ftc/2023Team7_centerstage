package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.lang.Math;
import java.util.List;

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

public class StarterAutoArmless extends LinearOpMode {
    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection
     * processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    final double ARMROTATEMAXVOLT = 1.1;// actually 1.102;
    final double ARMEXTENDEDMAXVOLT = 1.115;
    final double ARMROTATE0POSITION = 0.604;
    final double ARMROTATEMINVOLT = 0.191;// actually .084; then why not put .084? - Aidan

    final double VOLTSSTRINGUP = .935;
    final double VOLTSSTRINGDOWN = 1.335;// fiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiind!
    double lastTime;
    double startDecel = 30;

    double stopDecel = .5;

    static Pose fieldPose = new Pose(0, 0, 0);
    Pose velocityPose = new Pose(0, 0, 0);
    double zeroAngle = 0;
    double ticksPerRadian = 28.58 * (360 / (Math.PI * 2)); // found 28.58 by using testangular method to find ratio of
                                                           // ticks to radian
    public double inPerTick = 20 / 6786.0;
    static final double FEET_PER_METER = 3.28084;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    public final FtcDashboard dashboard = FtcDashboard.getInstance();
    // Yellow and Control Port 3
    public DcMotorEx frontLeft;
    // Green Control Port 0
    public DcMotorEx backLeft;
    // Red and Control Port 2
    public DcMotorEx frontRight;
    // White Control Port 1
    public DcMotorEx backRight;

    public IMU imu;
    public DcMotorEx deadPerp;

    public DcMotorEx deadLeft;

    public DcMotorEx deadRight;
    // Ex Hub port 2
    public DcMotor armMotor;
    // Ex Hub port 3
    public DcMotor stringMotor;
    // Ex Hub port 0
    public DcMotor lifterMotor;
    // Ex Hub port 1
    public DcMotor intakeMotor;
    // Control Hub Port 0
    public Servo armFlipper;
    double flipperPosition = 0;
    // Control Hub port 1
    public Servo finger;
    public Servo airplane;
    double[] servoPositions = { -1, .5, 1 };
    int currentPosition = 2;
    String increasingPosition = "increasing";
    public ColorSensor colorFR;
    public ColorSensor colorFL;
    public ColorSensor colorBR;
    public ColorSensor colorBL;
    public TouchSensor armuptouch;

    public AnalogInput armPot; // analog 0 control hub

    public AnalogInput stringPot; // analog 2 control hub

    double previousAngle = 0;

    double spinCounter = 0;
    int numFramesWithoutDetection = 0;
    int[] arrayDetections = new int[64];
    int detectionIndex = 0;
    double maxVelocity = 70.0;
    // Max forward velocity is 80 in/sec
    // Max strafe velocity is 70 in/sec

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
        double y = (deadLeft.getCurrentPosition() * inPerTick - deadRight.getCurrentPosition() * inPerTick) / 2;
        return new Pose(deadPerp.getCurrentPosition() * inPerTick, y,
                (imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle
                        - zeroAngle));
    }

    public void setPower(DcMotor motor, double targetPower, String name) {
        TelemetryPacket packet = new TelemetryPacket();
        double currentPower = motor.getPower();
        if (abs(targetPower) - .1 > abs(currentPower)) {

            double powerChange = 0.0125 * Math.signum(targetPower); // Adjust power based on the sign of targetPower

            double newPower = currentPower + powerChange;

            // Make sure newPower stays within the valid range of -1.0 to 1.0
            newPower = Math.max(-1.0, Math.min(1.0, newPower));

            motor.setPower(newPower);
            packet.put(name, newPower);
        } else {
            if (abs(targetPower) > 0 && abs(targetPower) < 0.2) {
                targetPower = Math.signum(targetPower) * .2;
            }
            motor.setPower(targetPower);
            packet.put(name, targetPower);
        }
        dashboard.sendTelemetryPacket(packet);
    }

    public void testAngular() {
        TelemetryPacket packet = new TelemetryPacket();
        frontRight.setPower(-1); // front
        frontLeft.setPower(1); // left
        backRight.setPower(-1); // right
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
        Point temp = new Point(0, 0);
        TelemetryPacket packet = new TelemetryPacket();
        Pose cur = fieldPose; // our current poe
        Pose diff = new Pose(target.x - cur.x, target.y - cur.y, wrap((target.angle) - (cur.angle))); // difference in
                                                                                                      // points
        packet.put("Diff", diff);
        // uses angles to find rotated X and Y
        double rotX = diff.x * Math.cos(-cur.angle) - diff.y * Math.sin(-cur.angle);
        double rotY = diff.x * Math.sin(-cur.angle) + diff.y * Math.cos(-cur.angle);
        double denom = Math.max(abs(rotX), abs(rotY));
        if (denom != 0) {
            rotX = rotX / denom;
            rotY = rotY / denom;
        }
        double angleFactor = diff.angle;
        double denominator = Math.max(abs(rotY) + abs(rotX) + abs(angleFactor), 1);
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double multiplier = deceleration(slowDown, diff.x, diff.y, diff.angle);
        packet.put("Multiplier", multiplier);
        double frontLeftPower = ((rotY + rotX - angleFactor) / denominator) * multiplier;
        double backLeftPower = ((rotY - rotX - angleFactor) / denominator) * multiplier;
        double frontRightPower = ((rotY - rotX + angleFactor) / denominator) * multiplier;
        double backRightPower = ((rotY + rotX + angleFactor) / denominator) * multiplier;

        // packet.put("frontleftPower",frontLeftPower);
        // packet.put("fronrightPower",frontRightPower);
        // packet.put("backrightpower",backRightPower);
        // packet.put("backleftPower",backLeftPower);
        // if(Math.abs(target.angle)==Math.PI/2){NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
        // frontLeftPower*=-1;
        // frontRightPower*=-1;
        // backLeftPower*=-1;
        // backRightPower*=-1;
        // }
        setPower(frontRight, frontRightPower, "frontRight");
        setPower(frontLeft, frontLeftPower, "frontLeft");
        setPower(backRight, backRightPower, "backRight");
        setPower(backLeft, backLeftPower, "backLeft");
        packet.put("cur", cur);
        dashboard.sendTelemetryPacket(packet);
        if (multiplier == 0) {
            return true;
        }
        return false;
    }

    void driveToPoint(Pose target, boolean slowDown) {
        boolean done = false;
        asyncPositionCorrector();
        while (!done && opModeIsActive()) {
            asyncPositionCorrector();
            done = driveToPointAsync(target, slowDown);
        }
    }

    protected void motorsStop() {
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    protected double deceleration(boolean slow, double rotX, double rotY, double angleDiff) {
        boolean slowDown = slow;
        double angleConstant = .1 / (10 * (Math.PI * 2) / 360);
        double d = Math.sqrt(((rotX * rotX) + (rotY * rotY)));// + Math.abs(angleDiff * angleConstant); Accounting for
                                                              // angle with distance

        if ((d <= stopDecel)) {
            return 0;
        }
        if (slowDown) {
            if (d < startDecel) {
                double powerLinear = (((1 - minimumPower) / (startDecel - stopDecel)) * (d - stopDecel) + minimumPower);
                double velocityRatio = (Math.sqrt((velocityPose.x * velocityPose.x) + (velocityPose.y * velocityPose.y))
                        / maxVelocity);
                return (powerLinear - velocityRatio);
            }
            // else if (targetspeed - speed > 0.02) {
            // //Motors would get faster
            // return 1;
            // }
            // else {
            // //Speed is normal
            // return 1;
            // }
            else if (d < .5) {
                return 0;

            } else if (d < 6) {
                return .5;
            }

            return 1;
        }
        return 1;
    }

    private double wrap(double theta) {
        double newTheta = theta;
        while (abs(newTheta) > Math.PI) {
            if (newTheta < -Math.PI) {
                newTheta += Math.PI * 2;
            } else {
                newTheta -= Math.PI * 2;
            }
        }
        return newTheta;
    }

    public void releasePixel() {
        intakeMotor.setPower(-.3);
        sleep(1000);
        intakeMotor.setPower(0);
    }

    public void unReleasePixel() {
        intakeMotor.setPower(.4);
        sleep(1000);
        intakeMotor.setPower(0);
    }

    private double positiveWrap(double theta) {
        double newTheta = theta;
        while (newTheta > Math.PI * 2) {
            newTheta -= Math.PI * 2;
        }
        while (newTheta < 0) {
            newTheta += Math.PI * 2;
        }
        return newTheta;
    }

    protected void initialize(Pose inputPose) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft"); // C3
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft"); // C0
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight"); // C2
        backRight = hardwareMap.get(DcMotorEx.class, "backRight"); // C1
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        deadLeft = hardwareMap.get(DcMotorEx.class, "backRight"); // C1
        deadRight = hardwareMap.get(DcMotorEx.class, "frontRight"); // C2
        deadPerp = hardwareMap.get(DcMotorEx.class, "backLeft"); // C0
        deadLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        sleep(500);
        // if robot takes zeroangle too much it breaks.
        zeroAngle = (imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle)
                - inputPose.angle;
    }

    void turnRobot(double angle) {
        TelemetryPacket packet = new TelemetryPacket();
        double difAngle = wrap(positiveWrap(angle) - positiveWrap(fieldPose.angle));
        double directionalSpeed;
        while (opModeIsActive() && !(abs(difAngle) < Math.toRadians(2))) {
            asyncPositionCorrector();
            difAngle = wrap(positiveWrap(angle) - positiveWrap(fieldPose.angle));
            directionalSpeed = -Math.signum(difAngle) * 0.5;
            packet.put("difangle", difAngle);
            dashboard.sendTelemetryPacket(packet);
            if (abs(difAngle) < 0.08 * Math.PI) {
                directionalSpeed *= 0.25;
            } else if (abs(difAngle) < 0.15 * Math.PI) {
                directionalSpeed *= 0.4;
            }

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

    protected void imuAngle() {
        telemetry.addData("IMU Angle", getCurrentPose().angle);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("IMU Angle", getCurrentPose().angle);
        dashboard.sendTelemetryPacket(packet);
    }

    protected boolean armAsync(double targVolt) {
        double armVolt = armPot.getVoltage();
        double armDif = targVolt - armVolt;
        TelemetryPacket packet = new TelemetryPacket();

        double multi = .75;
        packet.put("armDif", armDif);
        packet.put("targVolt", targVolt);
        // if((targVolt > ARMROTATEMAXVOLT) || (targVolt < ARMROTATEMINVOLT)){ // Just
        // incase
        // armMotor.setPower(0);
        // return true;
        // }
        if ((armVolt > ARMROTATEMAXVOLT) && (stringPot.getVoltage() > .98) && (armDif > 0)) {
            armMotor.setPower(0);
            return true;
        }
        if ((armVolt > ARMEXTENDEDMAXVOLT) && (armDif > 0)) {
            armMotor.setPower(0);
            return true;
        }
        if ((armVolt < ARMROTATEMINVOLT) && (armDif < 0)) {
            armMotor.setPower(0);
            return true;
        }
        if (abs(armDif) < .05) {
            armMotor.setPower(0);
            return true;
        }
        if (abs(armDif) < .1) {
            multi = multi * .5;
        }
        if (armDif > 0) {
            armMotor.setPower(multi);
        } else {
            multi = -multi;
            armMotor.setPower(multi);
        }
        packet.put("multi", multi);
        dashboard.sendTelemetryPacket(packet);
        return false;
    }

    protected boolean stringAsync(double targVolt) {
        // Positive power makes voltage go down
        double armDif = targVolt - stringPot.getVoltage();// -.3
        TelemetryPacket packet = new TelemetryPacket();
        double multi = .75;
        packet.put("stringDif", armDif);
        packet.put("currentVolt", stringPot.getVoltage());
        packet.put("targVoltString", targVolt);
        if (abs(ARMROTATE0POSITION - armPot.getVoltage()) > .98) {
            stringMotor.setPower(0);
            packet.put("armDif", abs(ARMROTATE0POSITION - armPot.getVoltage()));
            dashboard.sendTelemetryPacket(packet);
            return true;
        }
        if ((targVolt > VOLTSSTRINGDOWN) || (targVolt < VOLTSSTRINGUP)) { // Just incase
            stringMotor.setPower(0);
            packet.put("armDif", "bro what");
            dashboard.sendTelemetryPacket(packet);
            return true;
        }
        // if(stringPot.getVoltage()>VOLTSSTRINGDOWN||stringPot.getVoltage()<VOLTSSTRINGUP){
        // stringMotor.setPower(0);
        // packet.put("armDif","Charlie");
        // dashboard.sendTelemetryPacket(packet);
        // return true;
        // }
        if (abs(armDif) < .02) {
            stringMotor.setPower(0);
            dashboard.sendTelemetryPacket(packet);
            return true;
        }
        if (abs(armDif) < .1) {
            multi = multi * .5;
        }
        if (armDif < 0) {
            stringMotor.setPower(multi);
        } else {
            multi = -multi;
            stringMotor.setPower(multi);
        }
        packet.put("multi", multi);
        dashboard.sendTelemetryPacket(packet);
        return false;
    }

    protected void armMove(double leftStickX) {
        armMotor.setPower(leftStickX);
    }

    protected void lift(boolean DpadUpPressed, boolean previousDpadUpPressed, boolean DpadDownPressed,
            boolean previousDpadDownPressed) {
        if (DpadUpPressed && previousDpadUpPressed) {
            lifterMotor.setPower(1);
        } else if (DpadDownPressed && previousDpadDownPressed) {
            lifterMotor.setPower(-1);
        } else {
            lifterMotor.setPower(0);
        }
    }

    protected void setFinger(double degree) {
        finger.setPosition(degree);
    }

    protected void flipArm(double rightStickY) {
        double flipperConstant = 10;
        if (flipperPosition < 2) {
            if (rightStickY < 0.1) {
                armFlipper.setPosition(flipperPosition);
            } else if (rightStickY > 0.1) {
                armFlipper.setPosition(flipperPosition += rightStickY * flipperConstant);
            }

        } else if (flipperPosition > 178) {
            if (rightStickY > 0.1) {
                armFlipper.setPosition(flipperPosition);
            } else if (rightStickY < 0.1) {
                armFlipper.setPosition(flipperPosition += rightStickY * flipperConstant);
            }
        } else {
            armFlipper.setPosition(flipperPosition += rightStickY * flipperConstant);
        }
    }

    protected void setFlipperPosition(int position) {
        armFlipper.setPosition(position);
    }

    protected void returnArm() {
        while ((!armAsync(ARMROTATE0POSITION)) && opModeIsActive()) {
            armFlipper.setPosition(.5);
        }
        while (!(stringAsync(VOLTSSTRINGUP)) && opModeIsActive()) {

        }
        setFlipperPosition(1);
        sleep(500);
        while (!(stringAsync(VOLTSSTRINGDOWN)) && opModeIsActive()) {

        }
        while ((!armAsync(ARMROTATEMINVOLT)) && opModeIsActive()) {

        }
        finger.setPosition(servoPositions[1]);
    }

    protected boolean lifterTicksAsync(int targTicks) {
        // -10037 40 degrees
        // 167 0 degrees
        // pos power pos ticks
        int difTicks = targTicks - lifterMotor.getCurrentPosition();
        if (abs(difTicks) < 350) {
            lifterMotor.setPower(0);
            return (true);
        }
        if (difTicks < 0) {
            lifterMotor.setPower(-1);
            return (false);
        } else {
            lifterMotor.setPower(1);
            return (false);
        }

    }

    protected void Forward() {
        frontLeft.setPower(.5);
        frontRight.setPower(.5);
        backLeft.setPower(.5);
        backRight.setPower(.5);
    }

    protected void pixelPlaceAuto(Location location, boolean isRight) {
        if (isRight) {
            if (location == Location.CENTER) {
                setFlipperPosition(1);
                // stringAsync();
                // armAsync();
                while ((!armAsync(0.6)) && opModeIsActive()) {

                }
                while (!(stringAsync(VOLTSSTRINGUP)) && opModeIsActive()) {
                }
                while (!(armAsync(ARMEXTENDEDMAXVOLT)) && opModeIsActive()) {
                    armFlipper.setPosition(.4);
                }
                armFlipper.setPosition(-.92);

                sleep(400);
                finger.setPosition(1);

            } else if (location == Location.LEFT) {
                setFlipperPosition(1);
                // stringAsync();
                // armAsync();
                while ((!armAsync(0.6)) && opModeIsActive()) {

                }
                while (!(stringAsync(VOLTSSTRINGUP)) && opModeIsActive()) {
                }
                while (!(armAsync(ARMEXTENDEDMAXVOLT)) && opModeIsActive()) {
                    armFlipper.setPosition(.4);
                }
                armFlipper.setPosition(-.92);

                sleep(400);
                finger.setPosition(1);

            } else {
                setFlipperPosition(1);
                // stringAsync();
                // armAsync();
                while ((!armAsync(0.6)) && opModeIsActive()) {

                }
                while (!(stringAsync(VOLTSSTRINGUP)) && opModeIsActive()) {

                }
                armFlipper.setPosition(.4);
                sleep(300);
                while (!(stringAsync(VOLTSSTRINGDOWN)) && opModeIsActive()) {

                }
                while (!(armAsync(ARMROTATEMAXVOLT)) && opModeIsActive()) {

                }
                sleep(300);
                armFlipper.setPosition(-.92);
                sleep(400);
                finger.setPosition(1);

            }

        }
        // Right
        else {
            if (location == Location.CENTER) {
                setFlipperPosition(1);
                // stringAsync();
                // armAsync();
                while (!(armAsync(0.6)) && opModeIsActive()) {

                }
                while (!(stringAsync(VOLTSSTRINGUP)) && opModeIsActive()) {
                }
                while (!(armAsync(ARMROTATEMINVOLT)) && opModeIsActive()) {
                    armFlipper.setPosition(.4);

                }
                armFlipper.setPosition(-.92);
                sleep(400);
                finger.setPosition(1);

            } else if (location == Location.RIGHT) {
                setFlipperPosition(1);
                // stringAsync();
                // armAsync();
                while (!armAsync(0.6) && opModeIsActive()) {

                }
                while (!(stringAsync(VOLTSSTRINGUP)) && opModeIsActive()) {
                }
                while (!(armAsync(ARMROTATEMINVOLT) && opModeIsActive())) {
                    armFlipper.setPosition(.4);

                }
                armFlipper.setPosition(-.92);

                sleep(400);
                finger.setPosition(1);

            } else {
                setFlipperPosition(1);
                // stringAsync();
                // armAsync();
                while (!armAsync(0.6) && opModeIsActive()) {

                }
                while (!(stringAsync(VOLTSSTRINGUP)) && opModeIsActive()) {

                }
                armFlipper.setPosition(.4);
                sleep(300);
                while (!(stringAsync(VOLTSSTRINGDOWN)) && opModeIsActive()) {

                }
                while (!(armAsync(ARMROTATEMINVOLT)) && opModeIsActive()) {

                }
                armFlipper.setPosition(-.92);
                sleep(400);
                finger.setPosition(1);

            }
        }
    }

    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        } // end for() loop

    }

    public void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    } // end method initTfod()

    // comment #2
    @Override
    public void runOpMode() {
    }

}