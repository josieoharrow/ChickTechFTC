package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

import java.util.Locale;

/**
 * Created by Robotics on 8/27/2017.
 */
public class TeleOpLibrary {

    RobotHardware robot;
    HardwareMap hardwareMap;
    Orientation angles;
    CommonLibrary cl;


    static final double LEFT_ARM_CLOSED = 0.72;
    static final double RIGHT_ARM_CLOSED = 0.28;
    static final double LEFT_ARM_OPEN = 0.07;
    static final double RIGHT_ARM_OPEN = 0.93;
    static final double RIGHT_ARM_MID = 0.57;
    static final double LEFT_ARM_MID = 0.45;

    static final double RAMP_SERVO_DOWN = 0.0;
    static final double RAMP_SERVO_UP = 1.0;
    static final double RELIC_ACTUATOR_DOWN = 1.0;
    static final double RELIC_ACTUATOR_UP = 0.0;

    public double positionalMovementFLPower = 0;
    public double positionalMovementFRPower = 0;
    public double positionalMovementRRPower = 0;
    public double positionalMovementRLPower = 0;
    public double clockwiseRotation = 0;
    public double counterclockwiseRotation = 0;

    final float ENCODER_TICKS_PER_ROTATION = 1152;
    final float LIFT_MOTOR_MAXIMUM_POSITION = ENCODER_TICKS_PER_ROTATION * 4;
    final float LIFT_MOTOR_MINIMUM_POSITION = -10;
    final float SPEED_REDUCTION_COEFFICIENT = .7f;

    boolean liftMotorResetButtonPressed = false;

    float liftMotorFluidMinimum = LIFT_MOTOR_MINIMUM_POSITION;
    float liftMotorFluidMaximum = LIFT_MOTOR_MINIMUM_POSITION;
    int liftMotorEncoderPositon = 0;

    public enum motor {

        FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, REAR_LEFT_MOTOR, REAR_RIGHT_MOTOR
    }


    public void init(HardwareMap hardwareMapSent) {

        hardwareMap = hardwareMapSent;
        robot = new RobotHardware();
        cl = new CommonLibrary();
        cl.init(hardwareMapSent);
        robot.init(hardwareMap);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorEncoderPositon = robot.liftMotor.getCurrentPosition();
    }

    public void initGyro() {

        robot.initGyro();
    }

    public void translateLeftStickToRotation(Gamepad gamepad1) {
        // Button: left joystick

        float HorizontalInput = Range.clip(gamepad1.left_stick_x, -1, 1);
       // double clockwise = scaleInput(HorizontalInput);
       // double counterClockwise = scaleInput(-HorizontalInput);
        clockwiseRotation = scaleInput(HorizontalInput);
        counterclockwiseRotation = scaleInput(-HorizontalInput);
      /*  robot.frontLeftMotor.setPower(Range.clip((clockwise + positionalMovementFLPower), -1, 1)); //move power settings independent for clarity temporary fix
        robot.frontRightMotor.setPower(Range.clip((counterClockwise + positionalMovementFRPower), -1, 1));
        robot.rearRightMotor.setPower(Range.clip((counterClockwise + positionalMovementRRPower), -1, 1));
        robot.rearLeftMotor.setPower(Range.clip((clockwise + positionalMovementRLPower), -1, 1));*/
    }


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

        robot.frontLeftMotor.setPower(Range.clip((clockwiseRotation + positionalMovementFLPower) * speedCoefficient, -1, 1));
        robot.frontRightMotor.setPower(Range.clip((counterclockwiseRotation + positionalMovementFRPower) * speedCoefficient, -1, 1));
        robot.rearRightMotor.setPower(Range.clip((counterclockwiseRotation + positionalMovementRRPower) * speedCoefficient, -1, 1));
        robot.rearLeftMotor.setPower(Range.clip((clockwiseRotation + positionalMovementRLPower) * speedCoefficient, -1, 1));
    }


    public void translateRightStickToSlidingRelativeToRobot(Gamepad gamepad1) {

        float modifiedYValue = -gamepad1.right_stick_y; //This is because the phone was receiving y values that were flipped from all controllers, resulting in backwards driving behavior
        positionalMovementFLPower = scaleInput(Range.clip((modifiedYValue + gamepad1.right_stick_x), -1, 1)); //may need switched
        positionalMovementFRPower = scaleInput(Range.clip((modifiedYValue - gamepad1.right_stick_x), -1, 1));
        positionalMovementRRPower = scaleInput(Range.clip((modifiedYValue + gamepad1.right_stick_x), -1, 1));
        positionalMovementRLPower = scaleInput(Range.clip((modifiedYValue - gamepad1.right_stick_x), -1, 1));
        // robot.frontLeftMotor.setPower(flPower);
       // robot.frontRightMotor.setPower(frPower);
       // robot.rearRightMotor.setPower(rrPower);
       // robot.rearLeftMotor.setPower(rlPower);
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
        /*
        double flPower = scaleInput(Range.clip((modifiedThumbstickY + modifiedThumbstickX), -1, 1));
        double frPower = scaleInput(Range.clip((modifiedThumbstickY - modifiedThumbstickX), -1, 1));
        double rrPower = scaleInput(Range.clip((modifiedThumbstickY + modifiedThumbstickX), -1, 1));
        double rlPower = scaleInput(Range.clip((modifiedThumbstickY - modifiedThumbstickX), -1, 1));

        robot.frontLeftMotor.setPower(flPower);
        robot.frontRightMotor.setPower(frPower);
        robot.rearRightMotor.setPower(rrPower);
        robot.rearLeftMotor.setPower(rlPower);*/
    }


    public void generalTelemetry(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {

        telemetry.clear();
    /*    telemetry.addData("heading: ", robot.imu.getAngularOrientation().firstAngle);
        telemetry.addData("front right motor power ", robot.frontRightMotor.getPower());
        telemetry.addData("front left motor power ", robot.frontLeftMotor.getPower());
        telemetry.addData("rear right motor power ", robot.rearRightMotor.getPower());
        telemetry.addData("rear left motor power ", robot.rearLeftMotor.getPower());*/
        if (robot.liftMotorTouchSensor.getState() == true) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }

        telemetry.update();
    }



    public void armServos(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {

        if (gamepad2.x) {
            telemetry.addLine("Opening");
            telemetry.update();
            robot.leftArmServo.setPosition(LEFT_ARM_OPEN);
            robot.rightArmServo.setPosition(RIGHT_ARM_OPEN);
        }

        if (gamepad2.a) {

            telemetry.addLine("Mid Way");
            telemetry.update();
            robot.leftArmServo.setPosition(LEFT_ARM_MID);
            robot.rightArmServo.setPosition(RIGHT_ARM_MID);
        }
        if (gamepad2.b) {

            telemetry.addLine("Closing");
            telemetry.update();
            robot.leftArmServo.setPosition(LEFT_ARM_CLOSED);
            robot.rightArmServo.setPosition(RIGHT_ARM_CLOSED);
        }

        if (gamepad1.dpad_up) {

            telemetry.addLine("Opening");
            telemetry.update();
            robot.leftArmServo.setPosition(LEFT_ARM_OPEN);
            robot.rightArmServo.setPosition(RIGHT_ARM_OPEN);
        }
    }


    public void resetLiftMotorEncoderBasedOnTouchSensorActivation(Telemetry telemetry) {

        if (!robot.liftMotorTouchSensor.getState() == true) {

            liftMotorResetButtonPressed = true;
        } else if (liftMotorResetButtonPressed) {

            telemetry.addLine("Resetting Lift Motor Encoder Limits");
            telemetry.update();
            liftMotorFluidMaximum = robot.liftMotor.getCurrentPosition() + LIFT_MOTOR_MAXIMUM_POSITION;
            liftMotorFluidMinimum = robot.liftMotor.getCurrentPosition() + LIFT_MOTOR_MINIMUM_POSITION;
            liftMotorResetButtonPressed = false;
        }
    }

    public void setLiftMotorPower(Gamepad gamepad2, Telemetry telemetry) {

        float directionMultiplier = 0;
        float rightPower = gamepad2.right_trigger;
        float leftPower = gamepad2.left_trigger;
        float netPower = rightPower - leftPower;
        boolean ExceedingEncoderLimit = false;
        if (netPower != 0) {
            directionMultiplier = Math.abs(netPower) / netPower;
        }

        if (robot.liftMotor.getCurrentPosition() >= 0) {

            if (directionMultiplier == 1 && robot.liftMotor.getCurrentPosition() > liftMotorFluidMaximum) {
                ExceedingEncoderLimit = true;
            }
        } else {

            if (directionMultiplier == -1 && robot.liftMotor.getCurrentPosition() < liftMotorFluidMinimum) {
                ExceedingEncoderLimit = true;
            }
        }

        if(!ExceedingEncoderLimit) {

            telemetry.addData("Lift Motor encoder", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Upper limit", liftMotorFluidMaximum);
            telemetry.addData("Lower limit",liftMotorFluidMinimum);
            telemetry.addData("Direction multiplier", directionMultiplier);
            telemetry.addData("Power", gamepad2.right_trigger - gamepad2.left_trigger);
            telemetry.update();
            robot.liftMotor.setPower(Range.clip(scaleInput((double)(netPower)), -1, 1));
        } else {

            telemetry.addLine("WARNING: LIMIT HAS BEEN DETECTED");
            telemetry.addData("Lift Motor encoder", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Direction multiplier", directionMultiplier);
            telemetry.addData("Power", gamepad2.right_trigger - gamepad2.left_trigger);
            telemetry.update();
            if (!gamepad2.left_bumper) {
                robot.liftMotor.setPower(0);
            } else {
                robot.liftMotor.setPower(Range.clip(scaleInput((double)(netPower)), -1, 1));
            }
        }
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
