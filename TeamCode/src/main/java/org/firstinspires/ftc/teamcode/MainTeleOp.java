package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class MainTeleOp extends StarterAuto {
    public enum armState {
        Zeroing,
        Extending,
        Extended,

        Flipped,
        Intaking,

    }
    public enum returningArmState{
        Zeroing,
        Extending,
        Flipping,
        Flipped,
        Downing,
        Siding,



    }
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    armState currentArmState = armState.Zeroing;
    returningArmState currentReturningArmState = returningArmState.Zeroing;
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();

        initialize(new Pose(0, 0, 0));
        double robotVoltage = voltageSensor.getVoltage();
        double timeToFlip =2000;
        double currentFlipperPosition = 1;
        boolean stringingDown = false;
        double flipTimer = 0;
        double lastFlipPosition = 0;
        boolean flipped = false;
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
        downRed.setMode(DigitalChannel.Mode.OUTPUT);
        downGreen.setMode(DigitalChannel.Mode.OUTPUT);
        while (opModeIsActive()) {
            if(armFlipper.getPosition() == lastFlipPosition){
                if((flipTimer+timeToFlip) <System.currentTimeMillis()){
                    flipped=true;
                }
                lastFlipPosition = armFlipper.getPosition();
            }
            else{
                flipTimer=System.currentTimeMillis();
                flipped=false;
                if(lastFlipPosition-armFlipper.getPosition() <0){
                    timeToFlip=1300;
                }
                else{
                    timeToFlip=2000;
                }

                lastFlipPosition = armFlipper.getPosition();
            }
            packet.put("fliptimer", flipTimer);
            packet.put("flipposition",armFlipper.getPosition());
            packet.put("lastflipPosition",lastFlipPosition);
            packet.put("lifter", lifterMotor.getCurrentPosition());
            packet.put("Pixel 1 Color", pixel1sensor.alpha());
            packet.put("Pixel 2 Color", pixel2sensor.alpha());
            asyncPositionCorrector();
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
            double armYleftStick = -gamepad2.left_stick_y; // Remember, this is reversed!
            double armXleftStick = gamepad2.left_stick_x; // Counteract imperfect strafing
            double armYrightStick = -gamepad2.right_stick_y;
            double armXrightStick = gamepad2.right_stick_x;
            double driveXrightStick = gamepad1.right_stick_x;
            double driveYrightstick = gamepad1.right_stick_y;
            rx = -gamepad1.right_stick_x;
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
            boolean armDpadRight = cur2.dpad_right;
            boolean armDpadUp = cur2.dpad_up;
            boolean armDpadDown = cur2.dpad_down;
            boolean armDpadLeft = cur2.dpad_left;
            boolean previousArmDpadLeft = previousGamepad2.dpad_left;
            boolean armRightBumper = cur2.right_bumper;
            boolean armLeftBumper = cur2.left_bumper; // asynch holding send to mid arm, also add d pad contorl flipper,
            boolean previousArmLeftBumper = previousGamepad2.left_bumper;
            boolean previousArmRightBumper = previousGamepad2.right_bumper;
                                                      // arthy controls lifter with dpad
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
                rx = -Range.clip(gamepad1.right_stick_x, -0.25, 0.25);
            } else {
                driveYleftStick = Range.clip(-gamepad1.left_stick_y, -0.95, 0.95);
                rx = -Range.clip(gamepad1.right_stick_x, -0.75, 0.75);
            }
            packet.put("rx", rx);
            packet.put("cur", fieldPose);
            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -(current.angle - zeroAngle);
            if (!driveY && previousDriveY) {
                zeroAngle = current.angle;
            }
            if (armDpadRight) {

                lifterTicksAsync(-4363);
            } else {
                lifterMotor.setPower(0);
            }
            // Ask if we still want this or change be to be something useful
            if (driveB && !previousArmB) {
                fieldCentric = !fieldCentric;
            }
            if (!driveX && previousDriveX) {
                if(airplane.getPosition()!=.5){
                    airplane.setPosition(.5);
                }
                else{
                    airplane.setPosition(0);
                }

            }

            if (fieldCentric) {
                rotX = driveXleftStick * Math.cos(botHeading) - driveYleftStick * Math.sin(botHeading);
                rotY = driveXleftStick * Math.sin(botHeading) + driveYleftStick * Math.cos(botHeading);
            } else {
                rotX = driveXleftStick;
                rotY = driveYleftStick;
            }
            if ((armRightTrigger > 0.05) || (armLeftTrigger > 0.05)) {
                if (armRightTrigger > 0.05) {
                    stringAsync(stringPot.getVoltage() - .1);
                } else {
                    stringAsync(stringPot.getVoltage() + 0.1);
                }
            } else {
                stringMotor.setPower(0);
            }
            // Moves the arm back and forth
            if(armXleftStick>.05){
                armAsync(armPot.getVoltage()+.05);
            }
            else if(armXleftStick<-.05){
                armAsync(armPot.getVoltage()-.05);
            }
            else {
                armMotor.setPower(0);
            }
            if(armRightBumper){
                packet.put("ARMLEFTBUMPER", currentArmState);

                if(currentArmState == armState.Zeroing ){
                if(armAsync(ARMROTATE0POSITION)){
                    currentArmState = armState.Extending;
                }
                }
                if(currentArmState== armState.Extending ){
                    if(stringAsync(STRINGVOLTTOP)){
                        currentArmState = armState.Extended;
                    }
                }
                if(currentArmState == armState.Extended){
                    armFlipper.setPosition(FLIPPEROUT);
                    if(flipped){
                        currentArmState = armState.Flipped;
                    }
                }
                if(currentArmState == armState.Flipped){
                    if(armAsync(ARMEXTENDEDMAXVOLT)){
                        armFlipper.setPosition(-.92);
                    }
                }

            }
            else if(previousArmRightBumper && !armRightBumper){
                packet.put("ARMLEFTBUMPER", 0);
                armMotor.setPower(0);
                stringMotor.setPower(0);
                currentArmState = armState.Zeroing;
            }
            if(armLeftBumper){
                if(currentArmState == armState.Zeroing ){
                    if(armAsync(ARMROTATE0POSITION)){
                        currentArmState = armState.Extending;
                    }
                }
                if(currentArmState == armState.Extending ){
                    if(stringAsync(STRINGVOLTTOP)){
                        currentArmState = armState.Extended;
                    }
                }
                if(currentArmState == armState.Extended){
                    armFlipper.setPosition(FLIPPEROUT);
                    if(flipped){
                        currentArmState = armState.Flipped;
                    }
                }
                if(currentArmState == armState.Flipped){
                    if(armAsync(ARMROTATEMINVOLT)){
                        armFlipper.setPosition(-.92);
                    }
                }
            }
            else if(previousArmLeftBumper && !armLeftBumper){
                packet.put("ARMLEFTBUMPER", 0);
                armMotor.setPower(0);
                stringMotor.setPower(0);
                currentArmState = armState.Zeroing;
            }
            // if(!(armXleftStick<0.05 && armXleftStick>-0.05)){
            // if(armXleftStick>0.05){
            // armAsync(armPot.getVoltage()+.05);
            // }
            // else{
            // armAsync(armPot.getVoltage()-0.05);
            // }
            // }

            // Moves the strings out and in

            packet.put("ArmPot", armPot.getVoltage());
            packet.put("StringPot",
                    stringPot.getVoltage());
            // Moves the servo
            if ((armDpadUp || previousDpadUp || armDpadDown || previousDpadDown)) {
                lift(armDpadUp, previousDpadUp, armDpadDown, previousDpadDown);
            }
            if (armDpadLeft) {
                // Goes back to center
                packet.put("ARMCENTERER", currentReturningArmState);
                if(currentReturningArmState == returningArmState.Zeroing){
                    if(armFlipper.getPosition()<.5){
                        armFlipper.setPosition(FLIPPERPARTIAL);
                    }
                    if(armAsync(ARMROTATE0POSITION)){
                        currentReturningArmState = returningArmState.Extending;
                    }
                }
                else if(currentReturningArmState == returningArmState.Extending){
                    if(stringAsync(STRINGVOLTTOP)){
                        currentReturningArmState = returningArmState.Flipping;
                    }
                }
                else if(currentReturningArmState == returningArmState.Flipping){
                    armFlipper.setPosition(FLIPPERDOWN);
                    finger.setPosition(0);
                    currentReturningArmState = returningArmState.Flipped;

                }
                else if(currentReturningArmState == returningArmState.Flipped){
                    if(flipped){
                        currentReturningArmState = returningArmState.Downing;
                    }
                }
                else if(currentReturningArmState == returningArmState.Downing){
                    stringAsync(stringPot.getVoltage() + 0.06);


                }


            }
            if (!armDpadLeft && previousArmDpadLeft) {
                armMotor.setPower(0);
                stringMotor.setPower(0);
                currentReturningArmState = returningArmState.Zeroing;

            }
            if (armX && previousArmX) {
                intakeMotor.setPower(-1);
            } else if (armB && previousArmB) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }
            if (armA && !previousArmA) {
                if (Math.abs(stringPot.getVoltage()-STRINGVOLTDOWN)<.4) {
                    if (finger.getPosition() == FINGERCOVER) {
                        finger.setPosition(FINGERRIGHT);
                    } else {
                        finger.setPosition(FINGERCOVER);
                    }
                }
                else if(armPot.getVoltage()<ARMROTATE0POSITION){
                        if(finger.getPosition()==FINGERCOVER){
                            finger.setPosition(FINGERRIGHT);
                        }
                        else{
                            finger.setPosition(FINGERCOVER);
                        }
                }
                else{
                    if(finger.getPosition()==FINGERCOVER){
                        finger.setPosition(FINGERLEFT);
                    }
                    else{
                        finger.setPosition(FINGERCOVER);
                    }
                }

            }
            if (!armY && previousArmY) {
                if (armFlipper.getPosition() == .29) {
                    setFlipperPosition(-1);
                    currentFlipperPosition = -1;
                } else if (armFlipper.getPosition() <= 0) {
                    setFlipperPosition(.3);
                    currentFlipperPosition = .3;
                } else if (armFlipper.getPosition() == .3) {
                    setFlipperPosition(1);
                    currentFlipperPosition = 1;
                } else {
                    setFlipperPosition(.29);
                    currentFlipperPosition = .29;
                }
            }

            if(Math.abs(stringPot.getVoltage()-STRINGVOLTDOWN)<.1){
                downGreen.setState(true);
                downRed.setState(false);
            }
            else{
                downRed.setState(true);
                downGreen.setState(false);
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
            double s = deadLeft.getCurrentPosition();
            double t = deadRight.getCurrentPosition();
            double d = deadPerp.getCurrentPosition();
            packet.put("deadLeft", s);
            packet.put("deadRight", t);
            packet.put("deadPerp", d);
            packet.put("frontright", frontRightPower);
            packet.put("frontleftpower", frontLeftPower);
            packet.put("backrightpower", -backRightPower);
            packet.put("backleftpower", backLeftPower);
            frontRight.setPower(frontRightPower); // front
            frontLeft.setPower(frontLeftPower); // left
            backRight.setPower(-backRightPower); // right
            backLeft.setPower(backLeftPower); // back

            packet.put("zer", Math.toDegrees(zeroAngle));
            packet.put("imu x ", Math.toDegrees(current.angle));
            packet.put("imu y ", Math.toDegrees(
                    imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle));
            packet.put("imu z ", Math.toDegrees(
                    imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle));
            dashboard.sendTelemetryPacket(packet);
            try {
                previousGamepad1.copy(cur1);
                previousGamepad2.copy(cur2);
            } catch (RuntimeException e) {
            }
        }
    }
}