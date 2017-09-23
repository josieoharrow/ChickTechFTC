package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.RobotHardware;

/**
 * Created by Robotics on 8/27/2017.
 */
public class TeleOpLibrary {

    RobotHardware robot;
    HardwareMap hardwareMap;

    public void declareRobot(RobotHardware robotSent) {

        robot = robotSent;
    }

    public void init() {
        robot.init(hardwareMap);
    }

    public void motorsOn(){
        robot.frontMotor.setPower(1);
    }

    public void rotationJoystick(Gamepad gamepad1){
        // Button: left joystick
        float HorizontalInput = Range.clip(gamepad1.left_stick_x, -1, 1);
        double clockwise = scaleInput(HorizontalInput);
        double counterClockwise = scaleInput(-HorizontalInput);

        robot.leftMotor.setPower(clockwise);
        robot.frontMotor.setPower(clockwise);
        robot.rightMotor.setPower(counterClockwise);
        robot.backMotor.setPower(counterClockwise);
    }

    public void movementForwardJoystickTest(Gamepad gamepad1){
        // Button: right joystick
        float VerticalInput = Range.clip(gamepad1.right_stick_y, -1, 1);
        double frontPower = scaleInput(VerticalInput);
        double backPower = scaleInput(-VerticalInput);

        robot.frontMotor.setPower(frontPower);
        robot.backMotor.setPower(frontPower);
        robot.leftMotor.setPower(backPower);
        robot.rightMotor.setPower(backPower);

    }

    public void movementSidewaysJoystickTest(Gamepad gamepad1){
        //Button: left joystick
        float HorizontalInput = Range.clip(gamepad1.right_stick_x, -1, 1);
        double rightPower = scaleInput(HorizontalInput);
        double leftPower = scaleInput(-HorizontalInput);

        robot.leftMotor.setPower(rightPower);
        robot.rightMotor.setPower(rightPower);
        robot.frontMotor.setPower(leftPower);
        robot.backMotor.setPower(leftPower);
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
}
