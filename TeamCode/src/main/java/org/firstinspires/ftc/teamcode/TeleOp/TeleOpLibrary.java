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

    public void declareRobot(RobotHardware robotSent) {

        robot = robotSent;
    }

    public void init() {
        robot.init(hardwareMap);
    }

    public void translateLeftStickToRotation(Gamepad gamepad1){
        // Button: left joystick
        float HorizontalInput = Range.clip(gamepad1.left_stick_x, -1, 1);
        double clockwise = scaleInput(HorizontalInput) * .8;
        double counterClockwise = scaleInput(-HorizontalInput) * .8;

        robot.frontLeftMotor.setPower(counterClockwise);
        robot.frontRightMotor.setPower(clockwise);
        robot.rearRightMotor.setPower(clockwise);
        robot.rearLeftMotor.setPower(counterClockwise);
    }

    public void translateRightStickToSliding(Gamepad gamepad1) {

        double flPower = scaleInput(Range.clip((gamepad1.right_stick_y - gamepad1.right_stick_x), -1, 1));
        double frPower = scaleInput(Range.clip((gamepad1.right_stick_y + gamepad1.right_stick_x), -1, 1));
        double rrPower = scaleInput(Range.clip((gamepad1.right_stick_y - gamepad1.right_stick_x), -1, 1));
        double rlPower = scaleInput(Range.clip((gamepad1.right_stick_y + gamepad1.right_stick_x), -1, 1));
        robot.frontLeftMotor.setPower(flPower);
        robot.frontRightMotor.setPower(frPower);
        robot.rearRightMotor.setPower(rrPower);
        robot.rearLeftMotor.setPower(rlPower);

    }

    public void gyroTelemetry(Gamepad gamepad1, Telemetry telemetry) {
        telemetry.clear();
        telemetry.addData("Gyro Temperature ", robot.imu.getTemperature());
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("Movement Angle from throttle ", returnMovementAngleFromThrottleInput(gamepad1));
        telemetry.update();
    }

    public double returnMovementAngleFromThrottleInput(Gamepad gamepad1) {
        //Returns an angle with field orientation based on view from driver station as the far wall being y = 1,
        //the right wall as x = 1, the left as x = -1, and the closest wall as y = -1

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float currentHeading = Float.parseFloat(formatAngle(angles.angleUnit, angles.firstAngle)) + 90;


        if (currentHeading > 360) {

            currentHeading -= 360;
        }
        double angleInput = Math.atan(gamepad1.right_stick_y/ gamepad1.right_stick_x);

        double angleDirectionRelativeToCurrent = currentHeading - angleInput;//

        if (Math.abs(angleDirectionRelativeToCurrent) > 180) {

            if (angleDirectionRelativeToCurrent > 180) {

                angleDirectionRelativeToCurrent -= 360;
            } else {

                angleDirectionRelativeToCurrent += 360;
            }
        }

        return angleDirectionRelativeToCurrent;
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

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
