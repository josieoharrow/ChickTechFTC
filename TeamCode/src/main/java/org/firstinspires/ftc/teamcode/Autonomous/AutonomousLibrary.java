package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    Orientation startAngles;
    CommonLibrary cl;
    static VuforiaLocalizer vuforia;
    static String pictoKey = "unknown";
    static String vuMarkSeen = "no";
    static double SLOWING_INCHES_THRESHOLD = 10;
    static double DRIVING_POWER_SLOW_MODIFIER = 0.5;
    static float JEWEL_ACTUATOR_DOWN = 0.78f;
    static float JEWEL_ACTUATOR_UP = 0.2f;
    static double ENCODER_TICKS_TO_INCHES = 4 / 1130;
    static double INCHES_TO_ENCODER_TICKS = 288 / 4 * 0.1666666666; // * .31;

    public ColorSensor colorSensorREV;
    public int teamColorAndPosition = 0;

    public enum motor {

        FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, REAR_LEFT_MOTOR, REAR_RIGHT_MOTOR
    }

    public void declareRobot(RobotHardware robotSent) {

        //    robot = robotSent;
    }

    public void init(HardwareMap hardwareMapSent, Telemetry telemetry, Gamepad gamepad1, LinearOpMode caller) {

        setTeamColorAndPosition(gamepad1, telemetry, caller);
        telemetry.addLine("initializing");
        telemetry.update();
        hardwareMap = hardwareMapSent;
        robot = new RobotHardware();
        cl = new CommonLibrary();
        robot.init(hardwareMap);
        robot.initGyro();
        cl.init(hardwareMapSent);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.vuforiaLicenseKey = "Ac+j+R7/////AAAAGXEMop5pnkoqqEXMkOojnpQriKcyqCStGTQ0SVWtZDKiyucL+bWQPvA2YRrhGk/diKOkLGVRsP2l0UHYI37HSgl59Y81KNpEjxUEj34kk/Tm+ck3RrCgDuNtY4lsmePAuTAta6jakcmmESS4Gd2e0FAI97wuo6uJ4CAOXeAFs+AcqNQ162w10gJqOaTlYJVU1z8+UWQca/fwc/pcQ4sqwXzsL3NFpMgE3cijkAGxIZ6xAxkK5YI+3QJxzljDhszlG8dVOx8JJ4TflpzMNYpya36bPiKUlT++LQb6Xmn+HJpOChXg3vEtp2TV9hkFCe1CNjoYFCpsMTORho4tUGNPeUK0+JQBnHozcnbJdVnV+e/L";
        colorSensorREV = hardwareMap.get(ColorSensor.class, "jewel color sensor");
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        //lowerLift();
        resetLiftMotorEncoder(caller);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startAngles = robot.imu.getAngularOrientation();
        cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Open, robot);
    }

    public void moveLift(float rotations, LinearOpMode caller) {
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int oldEncoderPosition = robot.liftMotor.getCurrentPosition();
        final float ENCODER_TICKS_PER_ROTATION = 1152;

        if (rotations > 0) {

            while (!caller.isStopRequested() && robot.liftMotor.getCurrentPosition() < (rotations * ENCODER_TICKS_PER_ROTATION) + oldEncoderPosition) {

                float closeMultiplier = 1;
                if ((rotations * ENCODER_TICKS_PER_ROTATION + oldEncoderPosition) - robot.liftMotor.getCurrentPosition() < 200) {

                    closeMultiplier = 0.5f;
                }
                robot.liftMotor.setPower(.5 * closeMultiplier);
            }
        } else {

            while (!caller.isStopRequested() && robot.liftMotor.getCurrentPosition() > (rotations * ENCODER_TICKS_PER_ROTATION) + oldEncoderPosition) {

                float closeMultiplier = 1;
                if ((rotations * ENCODER_TICKS_PER_ROTATION + oldEncoderPosition) - robot.liftMotor.getCurrentPosition() > -200) {

                    closeMultiplier = 0.5f;
                }
                robot.liftMotor.setPower(-.5 * closeMultiplier);
            }
        }

        robot.liftMotor.setPower(0);
    }

    public int setTeamColorAndPosition(Gamepad gamepad1, Telemetry telemetry, LinearOpMode caller) {
        int teamColor = 0;
        int position = 0;
        String teamColorText = "";
        String positionText = "";
        try {

            while (teamColorAndPosition == 0 && !caller.isStarted()) { //Added opModeI

                if (teamColor + position == 0 || gamepad1.left_stick_button) {

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

                if (gamepad1.right_stick_button) {
                    teamColorAndPosition = teamColor + position;
                    telemetry.addData("team color", teamColorAndPosition);
                    telemetry.addLine(teamColorText + " | " + positionText);
                    telemetry.addLine("1 = Red Team, Corner Balance Board");
                    telemetry.addLine("2 = Red Team, Center Balance Board");
                    telemetry.addLine("3 = Blue Team, Corner Balance Board");
                    telemetry.addLine("4 = Blue Team, Center Balance Board");
                    telemetry.update();
                }
                //if (telemetry == null) {break;}
            }
        } catch (Exception e) {

            telemetry.addData("Initialization interrupted. \nERROR: ", e);
        }
        return teamColorAndPosition;
    }


    public String pictoDecipher(Telemetry telemetry, LinearOpMode caller) {
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        long startTime = System.currentTimeMillis();
        pictoKey = "unknown";

        while ("no".equals(vuMarkSeen) && !caller.isStopRequested()) { // While the vumark has not been seen

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("vumark", vuMark);
            telemetry.update();
            long currentTime = System.currentTimeMillis();
            double timeChange = (currentTime - startTime);
            // if (startTime < Thread.activeCount() + 2000 || caller.isStopRequested()) {break;}
            if (timeChange > 1000 || caller.isStopRequested()) {
                break;
            }
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { //If the pictograph is found
                telemetry.addData("I see something", 0);
                telemetry.update();
                if (vuMark == RelicRecoveryVuMark.LEFT) { //If the pictograph is the left pictograph
                    pictoKey = "left"; //Record that the pictograph is the left one
                    telemetry.addData("VuMark", "%s visible", vuMark); //Display which vumark has been seen

                    if ("left".equals(pictoKey)) { //See if it's been recorded that the pictograph is the left one
                        telemetry.addLine();
                        telemetry.addData("left", ""); //Write that it is the pictograph denoting left
                        telemetry.update();
                        break;
                    }
                }
                if (vuMark == RelicRecoveryVuMark.CENTER) { //If the pictograph is the center pictograph
                    pictoKey = "center"; //Record that the pictograph is the center one
                    telemetry.addData("VuMark", "%s visible", vuMark); //Display which vumark has been seen

                    if ("center".equals(pictoKey)) { //See if it's been recorded that the pictograph is the center one
                        telemetry.addLine();
                        telemetry.addData("center", ""); //Write that it is the pictograph denoting center
                        telemetry.update();
                        break;
                    }
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT) { //If the pictograph is the right pictograph
                    pictoKey = "right"; //Record that the pictograph is the right one
                    telemetry.addData("VuMark", "%s visible", vuMark); //Display which vumark has been seen

                    if ("right".equals(pictoKey)) { //See if it's been recorded that the pictograph is the right one
                        telemetry.addLine();
                        telemetry.addData("right", ""); //Write that it is the pictograph denoting right
                        telemetry.update();
                        break;
                    }

                }
                telemetry.update(); //Update the telemetry

            } else { //If the vumark isn't being seen
                telemetry.addData("VuMark", "is not visible"); //Show that the vumark hasn't been seen
                telemetry.update();
                if ("left".equals(pictoKey)) { //See if it's been recorded that the pictograph is the left one
                    telemetry.addLine();
                    telemetry.addData("left", ""); //Write that it is the pictograph denoting left
                    telemetry.update();
                    break;
                }
                if ("center".equals(pictoKey)) { //See if it's been recorded that the pictograph is the center one
                    telemetry.addLine();
                    telemetry.addData("center", ""); //Write that it is the pictograph denoting center
                    telemetry.update();
                    break;
                }
                if ("right".equals(pictoKey)) { //See if it's been recorded that the pictograph is the right one
                    telemetry.addLine();
                    telemetry.addData("right", ""); //Write that it is the pictograph denoting right
                    telemetry.update();
                    break;
                }
                telemetry.update();
//                if (startTime < Thread.activeCount() + 2000 || caller.isStopRequested()) {break;}
                if (timeChange > 1000 || caller.isStopRequested()) {
                    break;
                }
            }
        }
        telemetry.addData("Vuforia ", pictoKey);
        telemetry.update();
        return pictoKey;
    }

    public void driveAtAngle(double distance, double angle, Telemetry telemetry, LinearOpMode caller) {

        resetMotorEncoders(caller);

        double wheelPowerAngle = angle * Math.PI / 180;
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
        resetMotorEncoders(caller);
    }

    public void lowerLift(LinearOpMode caller) {

        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (!caller.isStopRequested() && robot.liftMotorTouchSensor.getState() == true) {

            robot.liftMotor.setPower(-.25);
        }
        robot.liftMotor.setPower(0);
    }


    public void driveUntilWallDetection(LinearOpMode caller) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontLeftMotor.setPower(0.4);
        robot.frontRightMotor.setPower(0.4);
        robot.rearRightMotor.setPower(0.4);
        robot.rearLeftMotor.setPower(0.4);

        int values[] = new int[2];
        values[0] = robot.mrRangeSensor.rawOptical();
        values[1] = robot.mrRangeSensor.rawUltrasonic();

        while ((values[1] < 9) && !caller.isStopRequested()) {

        }

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
    }


    public void driveByBlockColumns(LinearOpMode caller, Boolean left, int position) {
        /* for position, 1= closest column, 2 = middle, 3 = farthest */

        float distanceReadingOriginal = robot.mrRangeSensor.rawUltrasonic();
        float distanceReadingFluid = robot.mrRangeSensor.rawUltrasonic();
        int columnDetectedCount = 0;
        float front = 4;
        float middle = 10f;
        float end = 16f;//16
        float driveDistance;
        if (position == 1) {
            driveDistance = front;
        } else if (position == 2) {
            driveDistance = middle;
        } else {
            driveDistance = end;
        }
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (left) {
            robot.frontLeftMotor.setPower(0.4);
            robot.frontRightMotor.setPower(-0.4);
            robot.rearRightMotor.setPower(0.4);
            robot.rearLeftMotor.setPower(-0.4);
        } else {
            robot.frontLeftMotor.setPower(-0.4);
            robot.frontRightMotor.setPower(0.4);
            robot.rearRightMotor.setPower(-0.4);
            robot.rearLeftMotor.setPower(0.4);
        }


        while (!caller.isStopRequested()) {
            caller.telemetry.addData("MR ", robot.mrRangeSensor.getI2cAddress());
            caller.telemetry.addData("MR ", robot.mrRangeSensor.rawUltrasonic());
            caller.telemetry.addData("MR ", robot.mrRangeSensor.getDistance(DistanceUnit.CM));
            caller.telemetry.update();

            if (Math.abs(distanceReadingOriginal - distanceReadingFluid) > 3 && distanceReadingOriginal > distanceReadingFluid) {

                columnDetectedCount++;//is to check if columnCount is greater than columnDetectedCount. Not currently implemented.
                break;
            }
            distanceReadingFluid = robot.mrRangeSensor.rawUltrasonic();
        }

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);

        if (left) {

            driveAtAngle(driveDistance, 0, caller.telemetry, caller);
        } else {

            driveAtAngle(driveDistance, 180, caller.telemetry, caller);
        }
    }


    public void blockFollow(LinearOpMode caller, CommonLibrary cl) {

        double power = 0.65;
        double modifier = 0.25;
        double left = robot.leftSensorDistance.getLightDetected();
        double right = robot.rightSensorDistance.getLightDetected();
        double leftPower = 0;
        double rightPower = 0;
        boolean ranClearBlocks = false;
        while ((left < 0.6 && right < 0.6) && !caller.isStopRequested()) {
            leftPower = 0;
            rightPower = 0;
            if ((left > 0.15 || right > 0.15) && !ranClearBlocks) {//<??18
                // Sees block stage, clears other clocks
                //robot.blockGrabberServo.setPosition(BLOCK_GRABBER_CLOSED);
                //robot.leftArmServo.setPosition(LEFT_ARM_CLOSED);
                driveAtAngle(3, 90, caller.telemetry, caller);
                cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Mid, robot);
                //robot.leftArmServo.setPosition(LEFT_ARM_MID);
                driveAtAngle(1, 90, caller.telemetry, caller);
                ranClearBlocks = true;
            } else {
                left = robot.leftSensorDistance.getLightDetected();
                right = robot.rightSensorDistance.getLightDetected();
            }
            if (left > right + 0.02) {
                leftPower = -modifier;
            } else if (right > left + 0.02) {
                rightPower = -modifier;
            }
            if (leftPower + rightPower > 0) {
                caller.telemetry.addLine("Correcting path");
            } else {
                caller.telemetry.addLine("Not correcting path");
            }
            caller.telemetry.update();

            cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Mid, robot);
            //robot.leftArmServo.setPosition(LEFT_ARM_MID);
            robot.frontLeftMotor.setPower(power + leftPower);
            robot.frontRightMotor.setPower(power + rightPower);
            robot.rearRightMotor.setPower(power + leftPower);
            robot.rearLeftMotor.setPower(power + rightPower);
        }

        closeArmsAndRotateLift(cl, caller, 2);
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
    }



    public void resetMotorEncoders(LinearOpMode caller) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (!caller.isStopRequested() && robot.frontLeftMotor.isBusy() || robot.frontRightMotor.isBusy() || robot.rearLeftMotor.isBusy() || robot.rearRightMotor.isBusy()) {
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

        } else if (robot.colorSensorREV.blue() > robot.colorSensorREV.red()) {
            if (teamColorAndPosition == 1 || teamColorAndPosition == 2) {
                //drive opposite side of color sensor
                telemetry.addLine("I see the blue jewel and I am on red team");
                telemetry.update();
                PIDturnRelativeToField(10, telemetry, caller);
                robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                PIDturnRelativeToField(0, telemetry, caller);

            } else if (teamColorAndPosition == 3 || teamColorAndPosition == 4) {
                //drive side of color sensor
                telemetry.addLine("I see the blue jewel and I am on blue team");
                telemetry.update();
                PIDturnRelativeToField(-10, telemetry, caller);
                robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                PIDturnRelativeToField(0, telemetry, caller);

            } else {
                //For case when you don't know what team you are on- error with ground color sensor
                telemetry.addLine("IDK what team I'm on but I see the blue jewel");
                telemetry.update();
            }
        } else if (robot.colorSensorREV.blue() < robot.colorSensorREV.red()) {
            if (teamColorAndPosition == 1 || teamColorAndPosition == 2) {
                //drive side of color sensor
                telemetry.addLine("I see the red jewel and I am on red team");
                telemetry.update();
                PIDturnRelativeToField(-10, telemetry, caller);
                robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                PIDturnRelativeToField(0, telemetry, caller);

            } else if (teamColorAndPosition == 3 || teamColorAndPosition == 4) {
                //drive opposite side of color sensor
                telemetry.addLine("I see the red jewel and I am on blue team");
                telemetry.update();
                PIDturnRelativeToField(10, telemetry, caller);
                robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                PIDturnRelativeToField(0, telemetry, caller);

            } else {
                //For case when you don't know what team you are on- error with ground color sensor
                telemetry.addLine("IDK what team I'm on but I see the red jewel");
                telemetry.update();
            }
        }
        robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);

        telemetry.addLine("Ending Jewel knock off");
        telemetry.update();
    }

    public void closeArmsAndRotateLift(CommonLibrary cl, LinearOpMode caller, float liftRotations) {

        cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Close, robot);
        cl.wait(300, caller);
        moveLift(liftRotations, caller);
    }

    public void halfCloseArms(CommonLibrary cl, LinearOpMode caller) {

        cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Mid, robot);
        cl.wait(300, caller);
    }


    public void openArmsAndRotateLift(CommonLibrary cl, LinearOpMode caller, float liftRotations) {

        moveLift(liftRotations, caller);
        cl.wait(300, caller);
        cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Mid, robot);

    }

    public void lowerLiftAndSpinRollersOut(LinearOpMode caller) {

        lowerLift(caller);
        rollerSpinOut(1, caller);
    }

    public void rollersSpinIn(double timeInSeconds, Telemetry telemetry, LinearOpMode caller) {
        double spinStart = System.nanoTime();
        while (caller.opModeIsActive()) {
            telemetry.addLine("Suck in");
            telemetry.update();
            //robot.leftRoller.setPower(1);
            //robot.rightRoller.setPower(-1);
            robot.grabberRollers.setPower(1);   //This may need reversed
            if (System.nanoTime() > ((spinStart + (timeInSeconds * 1e9)))) {
                telemetry.addLine("Stop");
                telemetry.update();
                //robot.leftRoller.setPower(0);
                //robot.rightRoller.setPower(0);
                robot.grabberRollers.setPower(0);
                break;
            }
        }
    }

    public void setLiftToStay(LinearOpMode caller) {
        robot.liftMotor.setPower(0.025);
        /*int position = robot.liftMotor.getCurrentPosition();
        robot.liftMotor.setTargetPosition(position);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (!caller.isStopRequested() && robot.liftMotor.isBusy()) {

        }*/
    }

    public void rollerSpinOut(double timeInSeconds, LinearOpMode caller) {

        double spinStart = System.nanoTime();
        while (caller.opModeIsActive()){
            caller.telemetry.addLine("Spit out");
            caller.telemetry.update();
            //robot.leftRoller.setPower(-1);
            //robot.rightRoller.setPower(1);
            //robot.grabberRollers.setPower(-1);      //This may need to be reversed
            if (System.nanoTime() > ((spinStart + (timeInSeconds * 1e9)))) {
                caller.telemetry.addLine("Stop");
                caller.telemetry.update();
                //robot.leftRoller.setPower(0);
                //robot.rightRoller.setPower(0);
                robot.grabberRollers.setPower(0);
                break;
            }
        }
    }

    public void driveToVuforiaPositionFromTheLeft(Telemetry telemetry, LinearOpMode caller, String vuforiaPosition) {

        if (vuforiaPosition == "left") {
            driveAtAngle(4.5, 0, telemetry, caller);
        } else if (vuforiaPosition == "right") {
            driveAtAngle(16, 0, telemetry, caller);

        } else {
            //if center or unknown
            driveAtAngle(9.5, 0, telemetry, caller);

        }
        telemetry.addData("Position", vuforiaPosition);
        telemetry.update();
    }


    public void driveToVuforiaPositionFromTheRight(Telemetry telemetry, LinearOpMode caller, String vuforiaPosition) {

        if (vuforiaPosition == "right") {
            driveAtAngle(4.5, 180, telemetry, caller);
        } else if (vuforiaPosition == "left") {
            driveAtAngle(16, 180, telemetry, caller);

        } else {
            //if center or unknown
            driveAtAngle(9.5, 180, telemetry, caller);

        }
    }


    public double determineMotorTargetPositionRatio(double angleHeading, motor specifiedMotor) {

        double frontLeftMotorAngle = Math.PI / 4;
        double frontRightMotorAngle = -Math.PI / 4;
        double rearLeftMotorAngle = -Math.PI / 4;
        double rearRightMotorAngle = Math.PI / 4;

        double frontLeftMotorRatio = 1 / Math.sin(frontLeftMotorAngle + angleHeading);
        double frontRightMotorRatio = 1 / Math.sin(frontRightMotorAngle + angleHeading);
        double rearLeftMotorRatio = 1 / Math.sin(rearLeftMotorAngle + angleHeading);
        double rearRightMotorRatio = 1 / Math.sin(rearRightMotorAngle + angleHeading);

        if (specifiedMotor == motor.FRONT_LEFT_MOTOR) {
            return frontLeftMotorRatio;
        } else if (specifiedMotor == motor.FRONT_RIGHT_MOTOR) {
            return frontRightMotorRatio;
        } else if (specifiedMotor == motor.REAR_LEFT_MOTOR) {
            return rearLeftMotorRatio;
        } else {
            return rearRightMotorRatio;
        }
    }


    public void PIDturnRelativeToField(int angle, Telemetry telemetry, LinearOpMode caller) {

        runWithoutEncoders();
        if (angle >= 360) {
            angle = angle - 360;
        }
        if (angle <= -360) {
            angle = angle + 360;
        }
        double targetAngle = startAngles.firstAngle + angle;
        if (targetAngle >= 360) {
            targetAngle = targetAngle - 360;
        }
        if (targetAngle <= -360) {
            targetAngle = targetAngle + 360;
        }
        double acceptableError = 0.5;
        double currentError = 1;
        double power;
        while (Math.abs(currentError) > acceptableError && !caller.isStopRequested()) {

            angles = robot.imu.getAngularOrientation();
            double currentAngle = angles.firstAngle;
            currentError = targetAngle - currentAngle;
            telemetry.addData("Current angle", currentAngle);
            if (currentError > 180) {
                currentError = currentError - 360;
            }
            if (currentError <= -180) {
                currentError = currentError + 360;
            }
            telemetry.addLine();
            telemetry.addData("Current error", currentError);
            power = currentError * 0.01;
            if (power > 0.75) {
                power = 0.75;
            }
            if (power < 0.17 && power > 0) {
                power = 0.17;
            }
            if (power > -0.17 && power < 0) {
                power = -0.17;
            }
            if (power < -0.75) {
                power = -0.75;
            }
            telemetry.addLine();
            telemetry.addData("Power", power);
            telemetry.update();
            robot.frontLeftMotor.setPower(-power);
            robot.frontRightMotor.setPower(power);
            robot.rearRightMotor.setPower(power);
            robot.rearLeftMotor.setPower(-power);
            if (caller.isStopRequested()) {
                break;
            }
        }
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        telemetry.addData("done", "done");
        telemetry.update();
    }


    public void resetLiftMotorEncoder(LinearOpMode caller) {

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (!caller.isStopRequested() && robot.liftMotor.isBusy()) {
        }

    }

    public void whiskerDrive(Telemetry telemetry, LinearOpMode caller){
        float power;

        resetMotorEncoders(caller);
        while (robot.whiskerTouchLeft.getState() == false || robot.whiskerTouchRight.getState() == false){
            //if calculations are right, the red positions will be from the left so it will hit the right button
            //the blue side will drive to the right and will hit the left button

            power = 0.4f;
            if (teamColorAndPosition == 1 || teamColorAndPosition == 2){
                //Red team drives left
                robot.frontLeftMotor.setPower(-power);
                robot.frontRightMotor.setPower(power);
                robot.rearRightMotor.setPower(-power);
                robot.rearLeftMotor.setPower(power);
            } else if (teamColorAndPosition == 3 | teamColorAndPosition == 4){
                //Blue team drives right
                robot.frontLeftMotor.setPower(power);
                robot.frontRightMotor.setPower(-power);
                robot.rearRightMotor.setPower(power);
                robot.rearLeftMotor.setPower(-power);
            } else {
                 telemetry.addLine("I don't know who I am or where I am");
                 telemetry.update();
            }
            if (robot.frontLeftMotor.getCurrentPosition() * ENCODER_TICKS_TO_INCHES > 20 || robot.rearLeftMotor.getCurrentPosition() * ENCODER_TICKS_TO_INCHES >20 ){
                break;
            }

        }
        power = 0;
        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.rearRightMotor.setPower(power);
        robot.rearLeftMotor.setPower(power);
    }
}