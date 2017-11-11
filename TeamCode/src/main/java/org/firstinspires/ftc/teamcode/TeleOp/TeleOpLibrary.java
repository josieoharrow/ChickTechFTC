package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

import java.util.Locale;

/**
 * Created by Robotics on 8/27/2017.
 */
public class TeleOpLibrary {

    RobotHardware robot;
    HardwareMap hardwareMap;
    Orientation angles;

    public enum motor {

        FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, REAR_LEFT_MOTOR, REAR_RIGHT_MOTOR
    }


    public void init(HardwareMap hardwareMapSent) {

        hardwareMap = hardwareMapSent;
        robot = new RobotHardware();
        robot.init(hardwareMap);
    }


    public void translateLeftStickToRotation(Gamepad gamepad1) {
        // Button: left joystick

        float HorizontalInput = Range.clip(gamepad1.left_stick_x, -1, 1);
        double clockwise = scaleInput(HorizontalInput) * .8;
        double counterClockwise = scaleInput(-HorizontalInput) * .8;

        robot.frontLeftMotor.setPower(clockwise);
        robot.frontRightMotor.setPower(counterClockwise);
        robot.rearRightMotor.setPower(counterClockwise);
        robot.rearLeftMotor.setPower(clockwise);
    }


    public void translateRightStickToSlidingRelativeToRobot(Gamepad gamepad1) {

        float modifiedYValue = -gamepad1.right_stick_y; //This is because the phone was receiving y values that were flipped from all controllers, resulting in backwards driving behavior
        double flPower = scaleInput(Range.clip((modifiedYValue + gamepad1.right_stick_x), -1, 1)); //may need switched
        double frPower = scaleInput(Range.clip((modifiedYValue - gamepad1.right_stick_x), -1, 1));
        double rrPower = scaleInput(Range.clip((modifiedYValue + gamepad1.right_stick_x), -1, 1));
        double rlPower = scaleInput(Range.clip((modifiedYValue - gamepad1.right_stick_x), -1, 1));
        robot.frontLeftMotor.setPower(flPower);
        robot.frontRightMotor.setPower(frPower);
        robot.rearRightMotor.setPower(rrPower);
        robot.rearLeftMotor.setPower(rlPower);
    }


    public void translateRightStickToSlidingRelativeToField(Gamepad gamepad1, Telemetry telemetry) {

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = ((double)angles.firstAngle * Math.PI/180);
        double thumbstickAngle = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x);
        double thumbstickHypotenuse = Math.sqrt((gamepad1.right_stick_y * gamepad1.right_stick_y) + (gamepad1.right_stick_x * gamepad1.right_stick_x));
        double modifiedThumbstickAngle = thumbstickAngle - currentHeading;

        double modifiedThumbstickY = thumbstickHypotenuse * Math.sin(modifiedThumbstickAngle);
        double modifiedThumbstickX = thumbstickHypotenuse * Math.cos(modifiedThumbstickAngle);
        double flPower = scaleInput(Range.clip((modifiedThumbstickY + modifiedThumbstickX), -1, 1));
        double frPower = scaleInput(Range.clip((modifiedThumbstickY - modifiedThumbstickX), -1, 1));
        double rrPower = scaleInput(Range.clip((modifiedThumbstickY + modifiedThumbstickX), -1, 1));
        double rlPower = scaleInput(Range.clip((modifiedThumbstickY - modifiedThumbstickX), -1, 1));
        telemetry.update();
        robot.frontLeftMotor.setPower(flPower);
        robot.frontRightMotor.setPower(frPower);
        robot.rearRightMotor.setPower(rrPower);
        robot.rearLeftMotor.setPower(rlPower);
    }


    public void generalTelemetry(Gamepad gamepad1, Telemetry telemetry) {

        telemetry.clear();
        telemetry.addData("front right motor position ", robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("front left motor position ", robot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("rear right motor position ", robot.rearRightMotor.getCurrentPosition());
        telemetry.addData("rear left motor position ", robot.rearLeftMotor.getCurrentPosition());
        //telemetry.update();
    }


    private static double scaleInput(double dVal)  {
        /**
         * Converts raw input into values that can be used as power arguments for motors and servos
         */

        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        int index = (int) (dVal * 16.0);

        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale;

        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }


    String formatAngle(AngleUnit angleUnit, double angle) {

        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    

    String formatDegrees(double degrees) {

        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
