package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    static double ENCODER_TICKS_TO_INCHES = 4/1130;
    static double INCHES_TO_ENCODER_TICKS = 1130/4;

    public enum motor {

        FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, REAR_LEFT_MOTOR, REAR_RIGHT_MOTOR
    }

    public void declareRobot(RobotHardware robotSent) {

        robot = robotSent;
    }
    public void init(HardwareMap hardwareMapSent) {
        hardwareMap = hardwareMapSent;
        robot = new RobotHardware();
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.vuforiaLicenseKey = "Ac+j+R7/////AAAAGXEMop5pnkoqqEXMkOojnpQriKcyqCStGTQ0SVWtZDKiyucL+bWQPvA2YRrhGk/diKOkLGVRsP2l0UHYI37HSgl59Y81KNpEjxUEj34kk/Tm+ck3RrCgDuNtY4lsmePAuTAta6jakcmmESS4Gd2e0FAI97wuo6uJ4CAOXeAFs+AcqNQ162w10gJqOaTlYJVU1z8+UWQca/fwc/pcQ4sqwXzsL3NFpMgE3cijkAGxIZ6xAxkK5YI+3QJxzljDhszlG8dVOx8JJ4TflpzMNYpya36bPiKUlT++LQb6Xmn+HJpOChXg3vEtp2TV9hkFCe1CNjoYFCpsMTORho4tUGNPeUK0+JQBnHozcnbJdVnV+e/L";
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    }

    public void motorsOn() {

        robot.frontRightMotor.setPower(1);
        robot.frontLeftMotor.setPower(1);
        robot.rearLeftMotor.setPower(1);
        robot.rearRightMotor.setPower(1);
    }

    public void pictoDecipher(Telemetry telemetry, LinearOpMode caller){
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        int startTime = Thread.activeCount();

        while ("no".equals(vuMarkSeen)) { // While the vumark has not been seen

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (startTime < Thread.activeCount() + 2000 || caller.isStopRequested()) {break;}

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { //If the pictograph is found

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

            }

        }

    }

    public void driveAtAngle(double distance, double angle, Telemetry telemetry, LinearOpMode caller) {

        resetMotorEncoders();

        double wheelPowerAngle = angle * Math.PI /180;
        wheelPowerAngle = (Math.PI/2) - wheelPowerAngle;

        double xInput = Math.cos(wheelPowerAngle);
        double yInput = Math.sin(wheelPowerAngle);
        double flPower = Range.clip((yInput + xInput), -1, 1);
        double frPower = Range.clip((yInput - xInput), -1, 1);
        double rrPower = Range.clip((yInput + xInput), -1, 1);
        double rlPower = Range.clip((yInput - xInput), -1, 1);

        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + (int)(Math.abs(distance) * INCHES_TO_ENCODER_TICKS * determineMotorTargetPositionRatio(wheelPowerAngle, motor.FRONT_LEFT_MOTOR)));
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + (int)(Math.abs(distance) * INCHES_TO_ENCODER_TICKS * determineMotorTargetPositionRatio(wheelPowerAngle, motor.FRONT_RIGHT_MOTOR)));
        robot.rearLeftMotor.setTargetPosition(robot.rearLeftMotor.getCurrentPosition() + (int)(Math.abs(distance) * INCHES_TO_ENCODER_TICKS * determineMotorTargetPositionRatio(wheelPowerAngle, motor.REAR_LEFT_MOTOR)));
        robot.rearRightMotor.setTargetPosition(robot.rearRightMotor.getCurrentPosition() + (int)(Math.abs(distance) * INCHES_TO_ENCODER_TICKS * determineMotorTargetPositionRatio(wheelPowerAngle, motor.REAR_RIGHT_MOTOR)));

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

            telemetry.addData("front left", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("left power ", robot.frontLeftMotor.getPower());
            telemetry.addData("front right", robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("right power", robot.frontRightMotor.getPower());
            telemetry.addData("rear right", robot.rearRightMotor.getCurrentPosition());
            telemetry.addData("rear left", robot.rearLeftMotor.getCurrentPosition());
            telemetry.update();
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

    public void turnToAngle(int turnAngle, double speed, Telemetry telemetry){

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float stopTarget;
        boolean left = false;
        if (turnAngle >= 360){turnAngle = turnAngle - 360;}
        if (turnAngle <= -360){turnAngle = turnAngle + 360;}
        if (turnAngle < 0){left = true;}

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

    public void decipherJewelAndKnockOff() {
        double BallHue;
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        Color.RGBToHSV((int) (robot.colorSensorREV.red() * SCALE_FACTOR),
                (int) (robot.colorSensorREV.green() * SCALE_FACTOR),
                (int) (robot.colorSensorREV.blue() * SCALE_FACTOR),
                hsvValues);
        BallHue = hsvValues[0];

      //  if (BallHue >jfkdlsjfkljd)

    }

    public void setTeamColor() {
        //if values of color sensor = blue, then team color = blue
    }

    public void modernRoboticsSensorTest(Telemetry telemetry) {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        boolean bLedOn = true;
        robot.colorSensorMR.enableLed(bLedOn);

        Color.RGBToHSV(robot.colorSensorMR.red() * 8, robot.colorSensorMR.green() * 8, robot.colorSensorMR.blue() * 8, hsvValues);

        telemetry.addData("mr LED", bLedOn ? "On" : "Off");
        telemetry.addData("mr Clear", robot.colorSensorMR.alpha());
        telemetry.addData("mr Red  ", robot.colorSensorMR.red());
        telemetry.addData("mr Green", robot.colorSensorMR.green());
        telemetry.addData("mr Blue ", robot.colorSensorMR.blue());
        telemetry.addData("mr Hue", hsvValues[0]);
        telemetry.update();
    }

    public void revRoboticsColorSensorTest(Telemetry telemetry) {
       //set up for color sensor
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        //Actual steps in order to get values from color sensor
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (robot.colorSensorREV.red() * SCALE_FACTOR),
                (int) (robot.colorSensorREV.green() * SCALE_FACTOR),
                (int) (robot.colorSensorREV.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("rev Alpha", robot.colorSensorREV.alpha());
        telemetry.addData( "rev Red  ", robot.colorSensorREV.red());
        telemetry.addData("rev Green", robot.colorSensorREV.green());
        telemetry.addData("rev Blue ", robot.colorSensorREV.blue());
        telemetry.addData("rev Hue", hsvValues[0]);
        telemetry.update();
    }

    public double determineMotorTargetPositionRatio(double angleHeading, motor m){
        
        double frontLeftMotorAngle = 45;
        double frontRightMotorAngle = -45;
        double rearLeftMotorAngle = -45;
        double rearRightMotorAngle = 45;

        double frontLeftMotorRatio = Math.sin(frontLeftMotorAngle + angleHeading);
        double frontRightMotorRatio = Math.sin(frontRightMotorAngle + angleHeading);
        double rearLeftMotorRatio = Math.sin(rearLeftMotorAngle + angleHeading);
        double rearRightMotorRatio = Math.sin(rearRightMotorAngle + angleHeading);

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