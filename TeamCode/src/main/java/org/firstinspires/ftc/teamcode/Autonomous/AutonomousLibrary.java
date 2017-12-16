package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

import java.util.Locale;


/**
 * Created by Robotics on 8/27/2017.
 */
public class AutonomousLibrary {

    RobotHardware robot;
    HardwareMap hardwareMap;
    Orientation angles;
    static VuforiaLocalizer vuforia;
    static String pictoKey = "unknown";
    static String vuMarkSeen = "no";
    static double SLOWING_INCHES_THRESHOLD = 10;
    static double DRIVING_POWER_SLOW_MODIFIER = 0.5;
    static float JEWEL_ACTUATOR_DOWN = 0.9f;
    static float JEWEL_ACTUATOR_UP = 0.2f;
    static double ENCODER_TICKS_TO_INCHES = 4/1130;
    static double INCHES_TO_ENCODER_TICKS = 288/4 * 0.1666666666; // * .31;

    static final double LEFT_ARM_CLOSED = 0.72;
    static final double RIGHT_ARM_CLOSED = 0.28;
    static final double LEFT_ARM_OPEN = 0.07;
    static final double RIGHT_ARM_OPEN = 0.93;
    static final double RAMP_SERVO_DOWN = 0.0;
    static final double RAMP_SERVO_UP = 1.0;
    public ColorSensor colorSensorREV;
    public int teamColorAndPosition = 0;

    public enum motor {

        FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, REAR_LEFT_MOTOR, REAR_RIGHT_MOTOR
    }

    public void declareRobot(RobotHardware robotSent) {

        robot = robotSent;
    }
    public void init(HardwareMap hardwareMapSent, Telemetry telemetry, Gamepad gamepad1, LinearOpMode caller) {

        setTeamColorAndPosition(gamepad1, telemetry, caller);
        telemetry.addLine("initializing");
        telemetry.update();
        hardwareMap = hardwareMapSent;
        robot = new RobotHardware();
        CommonLibrary cl = new CommonLibrary();
        cl.init(hardwareMapSent);
        robot.init(hardwareMap);
        robot.initGyro();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.vuforiaLicenseKey = "Ac+j+R7/////AAAAGXEMop5pnkoqqEXMkOojnpQriKcyqCStGTQ0SVWtZDKiyucL+bWQPvA2YRrhGk/diKOkLGVRsP2l0UHYI37HSgl59Y81KNpEjxUEj34kk/Tm+ck3RrCgDuNtY4lsmePAuTAta6jakcmmESS4Gd2e0FAI97wuo6uJ4CAOXeAFs+AcqNQ162w10gJqOaTlYJVU1z8+UWQca/fwc/pcQ4sqwXzsL3NFpMgE3cijkAGxIZ6xAxkK5YI+3QJxzljDhszlG8dVOx8JJ4TflpzMNYpya36bPiKUlT++LQb6Xmn+HJpOChXg3vEtp2TV9hkFCe1CNjoYFCpsMTORho4tUGNPeUK0+JQBnHozcnbJdVnV+e/L";
        colorSensorREV = hardwareMap.get(ColorSensor.class, "jewel color sensor");
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        lowerLift();
        cl.resetLiftMotorEncoder();
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void moveLift(float rotations) {

        int oldEncoderPosition = robot.liftMotor.getCurrentPosition();
        final float ENCODER_TICKS_PER_ROTATION = 1152;

        if (rotations > 0) {

            while (robot.liftMotor.getCurrentPosition() < (rotations * ENCODER_TICKS_PER_ROTATION) + oldEncoderPosition) {

                float closeMultiplier = 1;
                if ((rotations * ENCODER_TICKS_PER_ROTATION + oldEncoderPosition) - robot.liftMotor.getCurrentPosition() < 200) {

                    closeMultiplier = 0.5f;
                }
                robot.liftMotor.setPower(.5 * closeMultiplier);
            }
        } else {

            while (robot.liftMotor.getCurrentPosition() > (rotations * ENCODER_TICKS_PER_ROTATION) + oldEncoderPosition) {

                float closeMultiplier = 1;
                if ((rotations * ENCODER_TICKS_PER_ROTATION + oldEncoderPosition) - robot.liftMotor.getCurrentPosition() > -200) {

                    closeMultiplier = 0.5f;
                }
               robot.liftMotor.setPower(-.5 * closeMultiplier);
            }
        }

        robot.liftMotor.setPower(0);
    }

    public int setTeamColorAndPosition (Gamepad gamepad1, Telemetry telemetry, LinearOpMode caller) {
        int teamColor = 0;
        int position = 0;
        String teamColorText = "";
        String positionText = "";

        while (teamColorAndPosition == 0 && !caller.isStarted()) {
            if (teamColor + position == 0 || gamepad1.left_stick_button){
                telemetry.addLine("Pick Team Color and Position:");
                telemetry.addLine("     Press X for Blue Team");
                telemetry.addLine("     Press B for Red Team");
                telemetry.addLine("     Press Right Bumper for Center Balance Board");
                telemetry.addLine("     Press Left Bumper for Corner Balance Board");
                telemetry.addLine("     Press Right Stick to Save and Leave");
                telemetry.addLine("");
                telemetry.addLine(teamColorText + " | " + positionText);
                telemetry.update();
            }
            if (gamepad1.x) {
                teamColor = 3;
                teamColorText = "Blue Team";
                telemetry.addLine(teamColorText + " | " + positionText);
                telemetry.addLine("For Menu, Press Left Stick");
                telemetry.addLine("To Save and Leave, Press Right Stick");
                telemetry.update();
            }
            if (gamepad1.b) {
                teamColor = 1;
                teamColorText = "Red Team";
                telemetry.addLine(teamColorText + " | " + positionText);
                telemetry.addLine("For Menu, Press Left Stick");
                telemetry.addLine("To Save and Leave, Press Right Stick");
                telemetry.update();
            }
            if (gamepad1.right_bumper) {
                position = 1;
                positionText = "Center Balance Board";
                telemetry.addLine(teamColorText + " | " + positionText);
                telemetry.addLine("For Menu, Press Left Stick");
                telemetry.addLine("To Save and Leave, Press Right Stick");
                telemetry.update();
            }
            if (gamepad1.left_bumper) {
                position = 0;
                positionText = "Corner Balance Board";
                telemetry.addLine(teamColorText + " | " + positionText);
                telemetry.addLine("For Menu, Press Left Stick");
                telemetry.addLine("To Save and Leave, Press Right Stick");
                telemetry.update();
            }

            if (gamepad1.right_stick_button)  {
                teamColorAndPosition = teamColor + position;
                telemetry.addData("team color", teamColorAndPosition);
                telemetry.addLine(teamColorText + " | " + positionText);
                telemetry.addLine("1 = Red Team, Corner Balance Board");
                telemetry.addLine("2 = Red Team, Center Balance Board");
                telemetry.addLine("3 = Blue Team, Corner Balance Board");
                telemetry.addLine("4 = Blue Team, Center Balance Board");
                telemetry.update();
            }
        }
        return teamColorAndPosition;
    }

    public void motorsOn() {

        robot.frontRightMotor.setPower(1);
        robot.frontLeftMotor.setPower(1);
        robot.rearLeftMotor.setPower(1);
        robot.rearRightMotor.setPower(1);
    }

    public String pictoDecipher(Telemetry telemetry, LinearOpMode caller){
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        long startTime =  System.currentTimeMillis();
        pictoKey = "unknown";

        while ("no".equals(vuMarkSeen)) { // While the vumark has not been seen

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("vumark", vuMark);
            telemetry.update();
            long currentTime = System.currentTimeMillis();
            double timeChange = (currentTime - startTime);
           // if (startTime < Thread.activeCount() + 2000 || caller.isStopRequested()) {break;}
            if (timeChange > 1000 || caller.isStopRequested()) {break;}

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { //If the pictograph is found
                telemetry.addData("I see something", 0);
                telemetry.update();
                if (vuMark == RelicRecoveryVuMark.LEFT) { //If the pictograph is the left pictograph
                    pictoKey = "left"; //Record that the pictograph is the left one
                    telemetry.addData("VuMark", "%s visible", vuMark); //Display which vumark has been seen

                    if ("left".equals(pictoKey)){ //See if it's been recorded that the pictograph is the left one
                        telemetry.addLine();
                        telemetry.addData("left", ""); //Write that it is the pictograph denoting left
                        telemetry.update();
                        break;
                    }
                }

                if (vuMark == RelicRecoveryVuMark.CENTER) { //If the pictograph is the center pictograph
                    pictoKey = "center"; //Record that the pictograph is the center one
                    telemetry.addData("VuMark", "%s visible", vuMark); //Display which vumark has been seen

                    if ("center".equals(pictoKey)){ //See if it's been recorded that the pictograph is the center one
                        telemetry.addLine();
                        telemetry.addData("center", ""); //Write that it is the pictograph denoting center
                        telemetry.update();
                        break;
                    }
                }

                if (vuMark == RelicRecoveryVuMark.RIGHT) { //If the pictograph is the right pictograph
                    pictoKey = "right"; //Record that the pictograph is the right one
                    telemetry.addData("VuMark", "%s visible", vuMark); //Display which vumark has been seen

                    if ("right".equals(pictoKey)){ //See if it's been recorded that the pictograph is the right one
                        telemetry.addLine();
                        telemetry.addData("right", ""); //Write that it is the pictograph denoting right
                        telemetry.update();
                        break;
                    }

                }
                telemetry.update(); //Update the telemetry

            }
            else { //If the vumark isn't being seen
                telemetry.addData("VuMark", "is not visible"); //Show that the vumark hasn't been seen
                telemetry.update();
                if ("left".equals(pictoKey)){ //See if it's been recorded that the pictograph is the left one
                    telemetry.addLine();
                    telemetry.addData("left", ""); //Write that it is the pictograph denoting left
                    telemetry.update();
                    break;
                }
                if ("center".equals(pictoKey)){ //See if it's been recorded that the pictograph is the center one
                    telemetry.addLine();
                    telemetry.addData("center", ""); //Write that it is the pictograph denoting center
                    telemetry.update();
                    break;
                }
                if ("right".equals(pictoKey)){ //See if it's been recorded that the pictograph is the right one
                    telemetry.addLine();
                    telemetry.addData("right", ""); //Write that it is the pictograph denoting right
                    telemetry.update();
                    break;
                }
                telemetry.update();
//                if (startTime < Thread.activeCount() + 2000 || caller.isStopRequested()) {break;}
                if (timeChange > 1000 || caller.isStopRequested()) {break;}

            }

        }
        telemetry.addData("Vuforia ", pictoKey);
        telemetry.update();
        return pictoKey;
    }

    public void driveAtAngle(double distance, double angle, Telemetry telemetry, LinearOpMode caller) {

        resetMotorEncoders();

        double wheelPowerAngle = angle * Math.PI /180;
        // wheelPowerAngle = (Math.PI/2) - wheelPowerAngle;

        double xInput = Math.cos(wheelPowerAngle);
        double yInput = Math.sin(wheelPowerAngle);
        double flPower = Range.clip((yInput + xInput), -1, 1);
        double frPower = Range.clip((yInput - xInput), -1, 1);
        double rrPower = Range.clip((yInput + xInput), -1, 1);
        double rlPower = Range.clip((yInput - xInput), -1, 1);

        double flMotorRatio = determineMotorTargetPositionRatio(wheelPowerAngle, motor.FRONT_LEFT_MOTOR);
        double frMotorRatio = determineMotorTargetPositionRatio(wheelPowerAngle, motor.FRONT_RIGHT_MOTOR);
        double rlMotorRatio = determineMotorTargetPositionRatio(wheelPowerAngle, motor.REAR_LEFT_MOTOR);
        double rrMotorRatio = determineMotorTargetPositionRatio(wheelPowerAngle, motor.REAR_RIGHT_MOTOR);

        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + (int) (Math.abs(distance) * INCHES_TO_ENCODER_TICKS * flMotorRatio));
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + (int) (Math.abs(distance) * INCHES_TO_ENCODER_TICKS * frMotorRatio));
        robot.rearLeftMotor.setTargetPosition(robot.rearLeftMotor.getCurrentPosition() + (int) (Math.abs(distance) * INCHES_TO_ENCODER_TICKS * rlMotorRatio));
        robot.rearRightMotor.setTargetPosition(robot.rearRightMotor.getCurrentPosition() + (int) (Math.abs(distance) * INCHES_TO_ENCODER_TICKS * rrMotorRatio));

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftMotor.setPower(flPower);
        robot.frontRightMotor.setPower(frPower);
        robot.rearRightMotor.setPower(rrPower);
        robot.rearLeftMotor.setPower(rlPower);

        //below should be && eventually and include all motors
        while ((robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy()) && !caller.isStopRequested()) {
        }

        telemetry.addLine("done");
        telemetry.update();
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        runUsingEncoders();
        resetMotorEncoders();
    }

    public void lowerLift() {
        while(robot.liftMotorTouchSensor.getState() == true) {

            robot.liftMotor.setPower(-.25);
        }
        robot.liftMotor.setPower(0);
    }



    public void blockFollow(LinearOpMode caller) {

        double power = .5;
        caller.telemetry.addLine("in the method");
        caller.telemetry.update();
        while (/*robot.leftODS.getRawLightDetected() < 0.7 && robot.rightODS.getRawLightDetected() < 0.7 && */!caller.isStopRequested()) {
            robot.rightODS.enableLed(true);
            robot.leftODS.enableLed(true);
            caller.telemetry.addData("Right ODS ", robot.rightODS.getLightDetected() * 10);
            caller.telemetry.addData("Left ODS ", robot.leftODS.getLightDetected() * 10);
            caller.telemetry.update();

            double powerModifierRight = Range.clip((1 + ((robot.leftODS.getRawLightDetected() - robot.rightODS.getRawLightDetected()) * 10) * power), 0.1, 0.7);//* 20?
            double powerModifierLeft = Range.clip((1 + ((robot.rightODS.getRawLightDetected() - robot.leftODS.getRawLightDetected()) * 10) * power), 0.1, 0.7);
            robot.frontLeftMotor.setPower(powerModifierLeft);
            robot.frontRightMotor.setPower(powerModifierRight);
            robot.rearRightMotor.setPower(powerModifierLeft);
            robot.rearLeftMotor.setPower(powerModifierRight);
        }
    }

    double convertEncoderValuesToLinearDrivingInches(double drivingAngle, double encoderValue) {

        double wheelToDrivingAngle = 45 - drivingAngle;
        double modifiedEncoderTicks = encoderValue * Math.cos(wheelToDrivingAngle);
        return convertEncoderTicksToInches(modifiedEncoderTicks);
    }

    double convertEncoderTicksToInches(double encoderTicks) {

        double rotationCount = encoderTicks / 1130;
        return rotationCount * Math.PI * 4; //this math is not correct
    }

    public void resetMotorEncoders() {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (robot.frontLeftMotor.isBusy() || robot.frontRightMotor.isBusy() || robot.rearLeftMotor.isBusy() || robot.rearRightMotor.isBusy()) {
        }
    }

    public void runUsingEncoders() {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoders() {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void motorEncoderTest(Telemetry telemetry)  {
        while (1 == 1) {
            telemetry.addData("Left Motor Position ", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Position ", robot.frontRightMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void turnToAngleWithGyro(int turnAngle, double speed, Telemetry telemetry){

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float stopTarget;
        boolean left = false;
        if (turnAngle >= 360){turnAngle = turnAngle - 360;}
        if (turnAngle <= -360){turnAngle = turnAngle + 360;}
        if (turnAngle < 0){left = true;}
        if (speed < 0){speed = Math.abs(speed);}

        if (left) {
            stopTarget = angles.firstAngle + turnAngle;
            while (angles.firstAngle > stopTarget) {
                robot.frontLeftMotor.setPower(speed);
                robot.frontRightMotor.setPower(-speed);
                robot.rearRightMotor.setPower(-speed);
                robot.rearLeftMotor.setPower(speed);
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
        else {
            stopTarget = angles.firstAngle + turnAngle;
            while (angles.firstAngle < stopTarget) {
                robot.frontLeftMotor.setPower(-speed);
                robot.frontRightMotor.setPower(speed);
                robot.rearRightMotor.setPower(speed);
                robot.rearLeftMotor.setPower(-speed);
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
    }

    public void turnToAngleWithEncoderTicks(int turnAngle, double turnSpeed, Telemetry telemetry){

        resetMotorEncoders();
        boolean left = false;
        if (turnAngle <= -360){ turnAngle = turnAngle + 360;}
        if (turnAngle >= 360){turnAngle = turnAngle - 360;}
        if (turnAngle < 0){left = true;}
        if (turnSpeed < 0){turnSpeed = Math.abs(turnSpeed);}
        float angleInInches = turnAngle * 0.314f;
        double flPower = turnSpeed;
        double frPower = turnSpeed;
        double rrPower = turnSpeed;
        double rlPower = turnSpeed;

        if (left){

            robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + (int) (INCHES_TO_ENCODER_TICKS * angleInInches));
            robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() - (int) (INCHES_TO_ENCODER_TICKS * angleInInches));
            robot.rearRightMotor.setTargetPosition(robot.rearRightMotor.getCurrentPosition() - (int) (INCHES_TO_ENCODER_TICKS * angleInInches));
            robot.rearLeftMotor.setTargetPosition(robot.rearLeftMotor.getCurrentPosition() + (int) (INCHES_TO_ENCODER_TICKS * angleInInches));

            frPower = -turnSpeed;
            rrPower = -turnSpeed;
        }
        else{
            robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() - (int) (INCHES_TO_ENCODER_TICKS * angleInInches));
            robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + (int) (INCHES_TO_ENCODER_TICKS * angleInInches));
            robot.rearRightMotor.setTargetPosition(robot.rearRightMotor.getCurrentPosition() + (int) (INCHES_TO_ENCODER_TICKS * angleInInches));
            robot.rearLeftMotor.setTargetPosition(robot.rearLeftMotor.getCurrentPosition() - (int) (INCHES_TO_ENCODER_TICKS * angleInInches));

            flPower = -turnSpeed;
            rlPower = -turnSpeed;
        }
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftMotor.setPower(flPower);
        robot.frontRightMotor.setPower(frPower);
        robot.rearRightMotor.setPower(rrPower);
        robot.rearLeftMotor.setPower(rlPower);
    }

    public void colorSensorTelemetry(Telemetry telemetry) {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.colorSensorREV.red() * SCALE_FACTOR),
                    (int) (robot.colorSensorREV.green() * SCALE_FACTOR),
                    (int) (robot.colorSensorREV.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Hue", hsvValues[0]);

            telemetry.addData("Blue ", robot.colorSensorREV.blue());
        telemetry.addData("Red ", robot.colorSensorREV.red());
        telemetry.addData("Green ", robot.colorSensorREV.green());
        telemetry.update();
    }

    public void turnToAngleWithPID(int angle, Telemetry telemetry, LinearOpMode caller){

        runWithoutEncoders();
        angles = robot.imu.getAngularOrientation();
        if (angle >= 360) {angle = angle - 360;}
        if (angle <= -360){angle = angle + 360;}
        double targetAngle = angles.firstAngle + angle;
        if (targetAngle > 180)  {targetAngle = targetAngle - 360;}
        if (targetAngle <= -180){targetAngle = targetAngle + 360;}
        double acceptableError = 0.5;
        double currentError = 1;
        double previousError = 0;
        double integral = 0;
        double power;
        double previousTime = 0;
        while (Math.abs(currentError) > acceptableError){

            double timeChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            timeChange = timeChange / 1e9;
            angles = robot.imu.getAngularOrientation();
            double currentAngle = angles.firstAngle ;
            currentError = targetAngle - currentAngle;
            if (currentError > 180)  {currentError = currentError - 360;}
            if (currentError <= -180){currentError = currentError + 360;}
            telemetry.addData("Current error", currentError);
            telemetry.addLine();
            telemetry.addData("Target angle", targetAngle);
            //integral = integral + currentError;
            double kpError = currentError * 0.008; //0.0042
            double kiIntegral = integral * 0.0002 * timeChange;
            double derivative = (currentError - previousError) / timeChange;
            double kdDerivative = derivative * 0;//WHAT IS THIS
            power = kpError + kiIntegral + kdDerivative;
            if (power > 1) {power = 1;}
            if (power < 0.13 && power > 0) {power = 0.13;}
            if (power > -0.13 && power < 0){power = -0.13;}
            if (power < -1){power = -1;}
            telemetry.addLine();
            telemetry.addData("Power", power);
            telemetry.update();
            robot.frontLeftMotor.setPower(-power);
            robot.frontRightMotor.setPower(power);
            robot.rearRightMotor.setPower(power);
            robot.rearLeftMotor.setPower(-power);
            previousError = currentError;
            if (caller.isStopRequested()){break;}
         }
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        telemetry.addData("done", "done");
        telemetry.update();
    }


    public void decipherJewelAndKnockOff(Telemetry telemetry, LinearOpMode caller, CommonLibrary cl) {

        telemetry.addLine("I have arrived to Jewel knock off");
        telemetry.update();

        robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_DOWN);

        cl.wait(1000, caller);

        telemetry.addLine("I am here");
        telemetry.update();

        if (Math.abs(robot.colorSensorREV.blue() - robot.colorSensorREV.red()) < 5) {
                telemetry.addLine("I have no idea what jewel I see");
                telemetry.update();

        } else if (robot.colorSensorREV.blue() > robot.colorSensorREV.red()){
            if (teamColorAndPosition == 1 || teamColorAndPosition == 2){
                //drive opposite side of color sensor
                telemetry.addLine("I see the blue jewel and I am on red team");
                telemetry.update();
                turnToAngleWithPID(10, telemetry, caller);
                robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                turnToAngleWithPID(-10, telemetry, caller);

            } else if (teamColorAndPosition == 3 || teamColorAndPosition == 4) {
                //drive side of color sensor
                telemetry.addLine("I see the blue jewel and I am on blue team");
                telemetry.update();
                turnToAngleWithPID(-10, telemetry, caller);
                robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                turnToAngleWithPID(10, telemetry, caller);

            } else {
                //For case when you don't know what team you are on- error with ground color sensor
                telemetry.addLine("IDK what team I'm on but I see the blue jewel");
                telemetry.update();
            }
        } else if (robot.colorSensorREV.blue() < robot.colorSensorREV.red()) {
            if (teamColorAndPosition == 1 || teamColorAndPosition == 2){
                //drive side of color sensor
                telemetry.addLine("I see the red jewel and I am on red team");
                telemetry.update();
                turnToAngleWithPID(-10, telemetry, caller);
                robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                turnToAngleWithPID(10, telemetry, caller);

            } else if (teamColorAndPosition == 3 || teamColorAndPosition == 4){
                //drive opposite side of color sensor
                telemetry.addLine("I see the red jewel and I am on blue team");
                telemetry.update();
                turnToAngleWithPID(10, telemetry, caller);
                robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                turnToAngleWithPID(-10, telemetry, caller);

            } else {
                //For case when you don't know what team you are on- error with ground color sensor
                telemetry.addLine("IDK what team I'm on but I see the red jewel");
                telemetry.update();
            }
        }
        telemetry.addLine("Ending Jewel knock off");
        telemetry.update();
    }


    public void closeArms(CommonLibrary cl, LinearOpMode caller) {

        robot.leftArmServo.setPosition(LEFT_ARM_CLOSED);
        robot.rightArmServo.setPosition(RIGHT_ARM_CLOSED);
        cl.wait(200, caller);
        Thread t1 = new Thread(new Runnable() {
            public void run() {

                moveLift(2);
            }
        });
        t1.start();
    }


    public void openArms() {

        moveLift(-1);
        robot.leftArmServo.setPosition(LEFT_ARM_OPEN);
        robot.rightArmServo.setPosition(RIGHT_ARM_OPEN);
    }


    public void driveToVuforiaPositionFromTheLeft(Telemetry telemetry, LinearOpMode caller, String vuforiaPosition) {

        if (vuforiaPosition == "left") {
            driveAtAngle(4, 0, telemetry, caller);
        } else if (vuforiaPosition == "right") {
            driveAtAngle(15, 0, telemetry, caller);

        } else {
            //if center or unknown
            driveAtAngle(9.5, 0, telemetry, caller);

        }
        telemetry.addData("Position", vuforiaPosition);
        telemetry.update();
    }


    public void driveToVuforiaPositionFromTheRight(Telemetry telemetry, LinearOpMode caller, String vuforiaPosition) {

        if (vuforiaPosition == "right") {
            driveAtAngle(4, 180, telemetry, caller);
        } else if (vuforiaPosition == "left") {
            driveAtAngle(15, 180, telemetry, caller);

        } else {
            //if center or unknown
            driveAtAngle(9.5, 180, telemetry, caller);

        }
    }


    public double determineMotorTargetPositionRatio(double angleHeading, motor m){

        double frontLeftMotorAngle = Math.PI/4;
        double frontRightMotorAngle = -Math.PI/4;
        double rearLeftMotorAngle = -Math.PI/4;
        double rearRightMotorAngle = Math.PI/4;

        double frontLeftMotorRatio = 1 / Math.sin(frontLeftMotorAngle + angleHeading);
        double frontRightMotorRatio = 1 / Math.sin(frontRightMotorAngle + angleHeading);
        double rearLeftMotorRatio = 1 / Math.sin(rearLeftMotorAngle + angleHeading);
        double rearRightMotorRatio = 1 / Math.sin(rearRightMotorAngle + angleHeading);

        if (m == motor.FRONT_LEFT_MOTOR) {
            return  frontLeftMotorRatio;
        } else if (m == motor.FRONT_RIGHT_MOTOR) {
            return frontRightMotorRatio;
        } else if (m == motor.REAR_LEFT_MOTOR) {
            return rearLeftMotorRatio;
        } else {
            return rearRightMotorRatio;
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}