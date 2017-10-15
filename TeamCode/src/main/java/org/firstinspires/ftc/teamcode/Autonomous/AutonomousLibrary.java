package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

/**
 * Created by Robotics on 8/27/2017.
 */
public class AutonomousLibrary {

    RobotHardware robot;
    HardwareMap hardwareMap;
    static VuforiaLocalizer vuforia;
    static String pictoKey = "unknown";
    static String vuMarkSeen = "no";
    static double SLOWING_INCHES_THRESHOLD = 10;
    static double DRIVING_POWER_SLOW_MODIFIER = 0.5;

    public static void initial (HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac+j+R7/////AAAAGXEMop5pnkoqqEXMkOojnpQriKcyqCStGTQ0SVWtZDKiyucL+bWQPvA2YRrhGk/diKOkLGVRsP2l0UHYI37HSgl59Y81KNpEjxUEj34kk/Tm+ck3RrCgDuNtY4lsmePAuTAta6jakcmmESS4Gd2e0FAI97wuo6uJ4CAOXeAFs+AcqNQ162w10gJqOaTlYJVU1z8+UWQca/fwc/pcQ4sqwXzsL3NFpMgE3cijkAGxIZ6xAxkK5YI+3QJxzljDhszlG8dVOx8JJ4TflpzMNYpya36bPiKUlT++LQb6Xmn+HJpOChXg3vEtp2TV9hkFCe1CNjoYFCpsMTORho4tUGNPeUK0+JQBnHozcnbJdVnV+e/L";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    }

    public void declareRobot(RobotHardware robotSent) {

        robot = robotSent;
    }

    public void init() {
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);
    }

    public void motorsOn(){
        robot.frontRightMotor.setPower(1);
    }

    public static void pictoDecipher(Telemetry telemetry){
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();

        while (vuMarkSeen == "no") { // While the vumark has not been seen

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { //If the pictograph is found

                if (vuMark == RelicRecoveryVuMark.LEFT) { //If the pictograph is the left pictograph
                    pictoKey = "left"; //Record that the pictograph is the left one
                    telemetry.addData("VuMark", "%s visible", vuMark); //Display which vumark has been seen

                    if (pictoKey == "left"){ //See if it's been recorded that the pictograph is the left one
                        telemetry.addLine();
                        telemetry.addData("left",""); //Write that it is the left one
                    }

                }

                if (vuMark == RelicRecoveryVuMark.CENTER) { //If the pictograph is the center pictograph
                    pictoKey = "center"; //Record that the pictograph is the center one
                    telemetry.addData("VuMark", "%s visible", vuMark); //Display which vumark has been seen

                    if (pictoKey == "center"){ //See if it's been recorded that the pictograph is the center one
                        telemetry.addLine();
                        telemetry.addData("center",""); //Write that it is the left one
                    }

                }

                if (vuMark == RelicRecoveryVuMark.RIGHT) { //If the pictograph is the right pictograph
                    pictoKey = "right"; //Record that the pictograph is the right one
                    telemetry.addData("VuMark", "%s visible", vuMark); //Display which vumark has been seen

                    if (pictoKey == "right"){ //See if it's been recorded that the pictograph is the right one
                        telemetry.addLine();
                        telemetry.addData("right",""); //Write that it is the left one
                    }

                }
                telemetry.update(); //Update the telemetry

            }
            else { //If the vumark isn't being seen
                telemetry.addData("VuMark", "is not visible"); //Show that the vumark hasn't been seen
                if (pictoKey == "left"){ //See if it's been recorded that the pictograph is the left one
                    telemetry.addLine();
                    telemetry.addData("left",""); //Write that it is the left one
                }
                if (pictoKey == "center"){ //See if it's been recorded that the pictograph is the center one
                    telemetry.addLine();
                    telemetry.addData("center",""); //Write that it is the left one
                }
                if (pictoKey == "right"){ //See if it's been recorded that the pictograph is the right one
                    telemetry.addLine();
                    telemetry.addData("right",""); //Write that it is the left one
                }
                telemetry.update();

            }

        }

    }

    public void driveAtAngle(double distance, double angle, Telemetry telemetry) {

        runUsingEncoders();
        double wheelPowerAngle = 90 - angle;

        double xInput = Math.cos(wheelPowerAngle);
        double yInput = Math.sin(wheelPowerAngle);
        double flPower = (Range.clip((yInput - xInput), -1, 1));
        double frPower = (Range.clip((yInput + xInput), -1, 1));
        double rrPower = (Range.clip((yInput - xInput), -1, 1));
        double rlPower = (Range.clip((yInput + xInput), -1, 1));
        //ok to here
        DcMotor motorToCheck = null;
        robot.frontLeftMotor.setPower(flPower);
        robot.frontRightMotor.setPower(frPower);
        robot.rearRightMotor.setPower(rrPower);
        robot.rearLeftMotor.setPower(rlPower);

        if (robot.frontLeftMotor.getPower() == 1) {
            motorToCheck = robot.frontLeftMotor;
        } else {
            motorToCheck = robot.frontRightMotor;
        }
        double distanceRemaining = Math.abs(Math.abs(convertEncoderTicksToInches((double) motorToCheck.getCurrentPosition())) - distance);
        telemetry.addData("distanc lef ", distanceRemaining);
        telemetry.update();
        
        while (distanceRemaining > SLOWING_INCHES_THRESHOLD) {

            distanceRemaining = Math.abs(convertEncoderTicksToInches((double)motorToCheck.getCurrentPosition())) - distance;
        }
        while (distanceRemaining > 0) {
            robot.frontLeftMotor.setPower(flPower * DRIVING_POWER_SLOW_MODIFIER);
            robot.frontRightMotor.setPower(frPower * DRIVING_POWER_SLOW_MODIFIER);
            robot.rearRightMotor.setPower(rrPower * DRIVING_POWER_SLOW_MODIFIER);
            robot.rearLeftMotor.setPower(rlPower * DRIVING_POWER_SLOW_MODIFIER);
            distanceRemaining = Math.abs(convertEncoderTicksToInches((double)motorToCheck.getCurrentPosition())) - distance;
        }

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        resetMotorEncoders();
    }

    double convertEncoderValuesToLinearDrivingInches(double drivingAngle, double encoderValue) {

        double wheelToDrivingAngle = 45 - drivingAngle;
        double modifiedEncoderTicks = encoderValue * Math.cos(wheelToDrivingAngle);
        return convertEncoderTicksToInches(modifiedEncoderTicks);
    }

    double convertEncoderTicksToInches(double encoderTicks) {

        double rotationCount = encoderTicks / 1440;
        return rotationCount * Math.PI * 4;
    }

    public void resetMotorEncoders() {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders() {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void RunWithoutEncoders() {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}


