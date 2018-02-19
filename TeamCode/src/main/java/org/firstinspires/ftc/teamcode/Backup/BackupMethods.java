package org.firstinspires.ftc.teamcode.Backup;

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
public class BackupMethods {

    RobotHardware robot = null;
    HardwareMap hardwareMap;
    Orientation angles;
    Orientation startAngles;
    CommonLibrary cl;
    static VuforiaLocalizer vuforia;
    static String pictoKey = "unknown";
    static String vuMarkSeen = "no";
    static double SLOWING_INCHES_THRESHOLD = 10;
    static double DRIVING_POWER_SLOW_MODIFIER = 0.5;
    static float JEWEL_ACTUATOR_DOWN = 0.8f;
    static float JEWEL_ACTUATOR_UP = 0.5f;
    static double ENCODER_TICKS_TO_INCHES = 4 / 1130;
    static double INCHES_TO_ENCODER_TICKS = 288 / 4 * 0.1666666666; // * .31;

    /*static final double LEFT_ARM_CLOSED = 0.72;
    static final double RIGHT_ARM_CLOSED = 0.28;
    static final double LEFT_ARM_OPEN = 0.07;
    static final double RIGHT_ARM_OPEN = 0.93;
    static final double RIGHT_ARM_MID = 0.57;
    static final double LEFT_ARM_MID = 0.45;*/
    static final double BLOCK_GRABBER_OPEN = 0.0;
    static final double BLOCK_GRABBER_MID = 0.3;    //This was at 0.5
    static final double BLOCK_GRABBER_CLOSED = 1; //This was at 1

    static final double RAMP_SERVO_DOWN = 0.0;
    static final double RAMP_SERVO_UP = 1.0;
    static final double RELIC_ACTUATOR_DOWN = 1.0;
    static final double RELIC_ACTUATOR_UP = 0.0;
    final float RELIC_MOTOR_MINIMUM_POSITION = 10;
    boolean isGrabberClosed = false;
    int liftMotorEncoderPositon = 0;



    public void motorEncoderTest(Telemetry telemetry) {
        while (1 == 1) {
            telemetry.addData("Left Motor Position ", robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("Right Motor Position ", robot.frontRightMotor.getCurrentPosition());
            telemetry.update();
        }
    }

/*
    BELOW BACKUP FOR RELATIVE TO FIELD.

    public void setDrivingMotorPowers(Gamepad gamepad1, Telemetry telemetry) {

        double speedCoefficient = 1;

        if (gamepad1.right_bumper) {
            speedCoefficient = SPEED_REDUCTION_COEFFICIENT;
        }

        translateLeftStickToRotation(gamepad1);

        //if (gamepad1.left_bumper) {
        //    translateRightStickToSlidingRelativeToField(gamepad1, telemetry);
        //} else {
        translateRightStickToSlidingRelativeToRobot(gamepad1);
        //}

        robot.frontLeftMotor.setPower(Range.clip((clockwiseRotation + positionalMovementFLPower) * Math.abs((clockwiseRotation + positionalMovementFLPower)) * speedCoefficient, -1, 1));
        robot.frontRightMotor.setPower(Range.clip((counterclockwiseRotation + positionalMovementFRPower) * Math.abs((counterclockwiseRotation + positionalMovementFRPower)) * speedCoefficient, -1, 1));
        robot.rearRightMotor.setPower(Range.clip((counterclockwiseRotation + positionalMovementRRPower) * Math.abs((counterclockwiseRotation + positionalMovementRRPower)) * speedCoefficient, -1, 1));
        robot.rearLeftMotor.setPower(Range.clip((clockwiseRotation + positionalMovementRLPower) * Math.abs((clockwiseRotation + positionalMovementRLPower)) * speedCoefficient, -1, 1));
    }

    public void translateRightStickToSlidingRelativeToField(Gamepad gamepad1, Telemetry telemetry) {

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = ((double)angles.firstAngle * Math.PI/180);
        double thumbstickAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
        double thumbstickHypotenuse = Math.sqrt((gamepad1.right_stick_y * gamepad1.right_stick_y) + (gamepad1.right_stick_x * gamepad1.right_stick_x));
        double modifiedThumbstickAngle = thumbstickAngle - currentHeading;

        double modifiedThumbstickY = thumbstickHypotenuse * Math.sin(modifiedThumbstickAngle);
        double modifiedThumbstickX = thumbstickHypotenuse * Math.cos(modifiedThumbstickAngle);
        positionalMovementFLPower = scaleInput(Range.clip((modifiedThumbstickY + modifiedThumbstickX), -1, 1));
        positionalMovementFRPower = scaleInput(Range.clip((modifiedThumbstickY - modifiedThumbstickX), -1, 1));
        positionalMovementRRPower = scaleInput(Range.clip((modifiedThumbstickY + modifiedThumbstickX), -1, 1));
        positionalMovementRLPower = scaleInput(Range.clip((modifiedThumbstickY - modifiedThumbstickX), -1, 1));
    }
    */
    public void testSetSlidePower(Gamepad gamepad1) {
        robot.relicLiftMotor.setPower(gamepad1.right_stick_y);
    }



    public void turnToAngleWithGyro(LinearOpMode caller, int turnAngle, double speed, Telemetry telemetry) {

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float stopTarget;
        boolean left = false;
        if (turnAngle >= 360) {
            turnAngle = turnAngle - 360;
        }
        if (turnAngle <= -360) {
            turnAngle = turnAngle + 360;
        }
        if (turnAngle < 0) {
            left = true;
        }
        if (speed < 0) {
            speed = Math.abs(speed);
        }

        if (left) {
            stopTarget = angles.firstAngle + turnAngle;
            while (!caller.isStopRequested() && angles.firstAngle > stopTarget) {
                robot.frontLeftMotor.setPower(speed);
                robot.frontRightMotor.setPower(-speed);
                robot.rearRightMotor.setPower(-speed);
                robot.rearLeftMotor.setPower(speed);
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        } else {
            stopTarget = angles.firstAngle + turnAngle;
            while (!caller.isStopRequested() && angles.firstAngle < stopTarget) {
                robot.frontLeftMotor.setPower(-speed);
                robot.frontRightMotor.setPower(speed);
                robot.rearRightMotor.setPower(speed);
                robot.rearLeftMotor.setPower(-speed);
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        }
    }

    public void turnToAngleWithEncoderTicks(int turnAngle, double turnSpeed, Telemetry telemetry, LinearOpMode caller) {

        resetMotorEncoders(caller);
        boolean left = false;
        if (turnAngle <= -360) {
            turnAngle = turnAngle + 360;
        }
        if (turnAngle >= 360) {
            turnAngle = turnAngle - 360;
        }
        if (turnAngle < 0) {
            left = true;
        }
        if (turnSpeed < 0) {
            turnSpeed = Math.abs(turnSpeed);
        }
        float angleInInches = turnAngle * 0.314f;
        double flPower = turnSpeed;
        double frPower = turnSpeed;
        double rrPower = turnSpeed;
        double rlPower = turnSpeed;

        if (left) {

            robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + (int) (INCHES_TO_ENCODER_TICKS * angleInInches));
            robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() - (int) (INCHES_TO_ENCODER_TICKS * angleInInches));
            robot.rearRightMotor.setTargetPosition(robot.rearRightMotor.getCurrentPosition() - (int) (INCHES_TO_ENCODER_TICKS * angleInInches));
            robot.rearLeftMotor.setTargetPosition(robot.rearLeftMotor.getCurrentPosition() + (int) (INCHES_TO_ENCODER_TICKS * angleInInches));

            frPower = -turnSpeed;
            rrPower = -turnSpeed;
        } else {
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

    public void turnToAngleWithPID(int angle, Telemetry telemetry, LinearOpMode caller) {

        runWithoutEncoders();
        if (angle >= 360) {
            angle = angle - 360;
        }
        if (angle <= -360) {
            angle = angle + 360;
        }
        double targetAngle = angles.firstAngle + angle;
        if (targetAngle > 180) {
            targetAngle = targetAngle - 360;
        }
        if (targetAngle <= -180) {
            targetAngle = targetAngle + 360;
        }
        double acceptableError = 0.5;
        double currentError = 1;
        double previousError = 0;
        double integral = 0;
        double power;
        double previousTime = 0;
        while (!caller.isStopRequested() && Math.abs(currentError) > acceptableError) {

            double timeChange = System.nanoTime() - previousTime;
            previousTime = System.nanoTime();
            timeChange = timeChange / 1e9;
            angles = robot.imu.getAngularOrientation();
            double currentAngle = angles.firstAngle;
            currentError = targetAngle - currentAngle;
            if (currentError > 180) {
                currentError = currentError - 360;
            }
            if (currentError <= -180) {
                currentError = currentError + 360;
            }
            telemetry.addData("Current error", currentError);
            telemetry.addLine();
            telemetry.addData("Target angle", targetAngle);
            //integral = integral + currentError;
            double kpError = currentError * 0.008;
            double kiIntegral = integral * 0.0002 * timeChange;
            double derivative = (currentError - previousError) / timeChange;
            double kdDerivative = derivative * 0;//WHAT IS THIS
            power = kpError + kiIntegral + kdDerivative;
            if (power > 1) {
                power = 1;
            }
            if (power < 0.13 && power > 0) {
                power = 0.13;
            }
            if (power > -0.13 && power < 0) {
                power = -0.13;
            }
            if (power < -1) {
                power = -1;
            }
            telemetry.addLine();
            telemetry.addData("Power", power);
            telemetry.update();
            robot.frontLeftMotor.setPower(-power);
            robot.frontRightMotor.setPower(power);
            robot.rearRightMotor.setPower(power);
            robot.rearLeftMotor.setPower(-power);
            previousError = currentError;
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

    public void blockFollowTest(LinearOpMode caller, boolean stopAtBlock) {

        double power = .25;
        caller.telemetry.addLine("in the method");
        caller.telemetry.update();
        boolean going = true;

        while (going && !caller.isStopRequested()) {
            double left = robot.leftSensorDistance.getDistance(DistanceUnit.CM);
            double right = robot.rightSensorDistance.getDistance(DistanceUnit.CM);

            if ((left < 3 || right < 3) && stopAtBlock) {
                going = false;
            }
            caller.telemetry.addData("left Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.leftSensorDistance.getLightDetected()));
            caller.telemetry.addData("Right Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.rightSensorDistance.getLightDetected()));
            caller.telemetry.addData("Right ODS ", right);
            caller.telemetry.addData("Left ODS ", left);
            caller.telemetry.update();

            double powerModifierRight = Range.clip((1 + (left - right) * power), 0.1, 0.5);
            double powerModifierLeft = Range.clip((1 + (right - left) * power), 0.1, 0.5);
            robot.frontLeftMotor.setPower(powerModifierLeft);
            robot.frontRightMotor.setPower(powerModifierRight);
            robot.rearRightMotor.setPower(powerModifierLeft);
            robot.rearLeftMotor.setPower(powerModifierRight);
        }
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
    }


    double convertEncoderTicksToInches(double encoderTicks) {

        double rotationCount = encoderTicks / 1130;
        return rotationCount * Math.PI * 4; //this math is not correct
    }



    //support for unused
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

}