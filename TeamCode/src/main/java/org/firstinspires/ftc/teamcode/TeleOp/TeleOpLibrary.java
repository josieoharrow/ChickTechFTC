package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

/**
 * Created by Robotics on 8/27/2017.
 */
public class TeleOpLibrary {

    RobotHardware robot;
    HardwareMap hardwareMap;
    Orientation angles;
    TeleOpDriver mainDriver;
    TeleOpDriverTest testDriver;
    CommonLibrary cl;

    static final double RELIC_GRABBER_CLOSED = .12;
    static final double RELIC_GRABBER_OPEN = 0.32;
    static final double RELIC_ROTATE_DOWN = 0.9;
    static final double RELIC_ROTATE_UP = 0.1;

    public double positionalMovementFLPower = 0;
    public double positionalMovementFRPower = 0;
    public double positionalMovementRRPower = 0;
    public double positionalMovementRLPower = 0;
    public double clockwiseRotation = 0;
    public double counterclockwiseRotation = 0;

    final float ENCODER_TICKS_PER_ROTATION = 1152;
    final float LIFT_MOTOR_MAXIMUM_POSITION = ENCODER_TICKS_PER_ROTATION * 5;//was 4
    final float LIFT_MOTOR_MINIMUM_POSITION = -10;
    final float SPEED_REDUCTION_COEFFICIENT = .3f;

    final float RELIC_MOTOR_MAXIMUM_POSITION = ENCODER_TICKS_PER_ROTATION * 4;      //These are the same as the glyph lift so it probably needs changed

    float relicFluidMaximum = RELIC_MOTOR_MAXIMUM_POSITION;

    boolean liftMotorResetButtonPressed = false;

    boolean relicResetPressed = true;

    float liftMotorFluidMinimum = LIFT_MOTOR_MINIMUM_POSITION;
    float liftMotorFluidMaximum = LIFT_MOTOR_MINIMUM_POSITION;      //why is this this???

    public enum motor {

        FRONT_LEFT_MOTOR, FRONT_RIGHT_MOTOR, REAR_LEFT_MOTOR, REAR_RIGHT_MOTOR
    }


    public void init(OpMode caller) {

        try {
            mainDriver = (TeleOpDriver) caller;
        } catch (Exception e) {
            testDriver = (TeleOpDriverTest) caller;
        }

        hardwareMap = caller.hardwareMap;
        robot = new RobotHardware();
        cl = new CommonLibrary();
        cl.init(caller.hardwareMap);
        robot.init(hardwareMap);
    }


    public void initGyro() {

        robot.initGyro();
    }


    public void translateLeftStickToRotation(Gamepad gamepad1) {
        // Button: left joystick

        float HorizontalInput = Range.clip(gamepad1.left_stick_x, -1, 1);
        clockwiseRotation = scaleInput(HorizontalInput);
        counterclockwiseRotation = scaleInput(-HorizontalInput);
    }


    public void testMotors(TeleOpDriverTest caller) {

        robot.frontLeftMotor.setPower(caller.gamepad1.right_stick_y);
        robot.frontRightMotor.setPower(caller.gamepad1.right_stick_y);
        robot.rearLeftMotor.setPower(caller.gamepad1.right_stick_y);
        robot.rearRightMotor.setPower(caller.gamepad1.right_stick_y);

        caller.telemetry.addData("Mode ", robot.rearLeftMotor.getMode());
        caller.telemetry.addData("Power ", robot.rearLeftMotor.getPower());
        caller.telemetry.addData("zero left rear ", robot.rearLeftMotor.getZeroPowerBehavior());
        caller.telemetry.addData("zero right rear ", robot.rearRightMotor.getZeroPowerBehavior());
        caller.telemetry.update();
    }


    public void lowerLift() {

        while(robot.liftMotorTouchSensor.getState() && mainDriver.running) {

            robot.liftMotor.setPower(-.5);
        }

        robot.liftMotor.setPower(0);
        mainDriver.liftLowered = true;
    }


    public void setServoPosition(Servo servo, double position) {

        servo.setPosition(position);
    }




    public void setDrivingMotorPowers(Gamepad gamepad1, Telemetry telemetry) {

        double speedCoefficient = 1;

        if (gamepad1.right_bumper) {
            speedCoefficient = SPEED_REDUCTION_COEFFICIENT;
        }

        translateLeftStickToRotation(gamepad1);
        translateRightStickToSlidingRelativeToRobot(gamepad1);

        robot.frontLeftMotor.setPower(Range.clip((clockwiseRotation + positionalMovementFLPower) * Math.abs((clockwiseRotation + positionalMovementFLPower)) * speedCoefficient, -1, 1));
        robot.frontRightMotor.setPower(Range.clip((counterclockwiseRotation + positionalMovementFRPower) * Math.abs((counterclockwiseRotation + positionalMovementFRPower)) * speedCoefficient, -1, 1));
        robot.rearRightMotor.setPower(Range.clip((counterclockwiseRotation + positionalMovementRRPower) * Math.abs((counterclockwiseRotation + positionalMovementRRPower)) * speedCoefficient, -1, 1));
        robot.rearLeftMotor.setPower(Range.clip((clockwiseRotation + positionalMovementRLPower) * Math.abs((clockwiseRotation + positionalMovementRLPower)) * speedCoefficient, -1, 1));
    }


    public void translateRightStickToSlidingRelativeToRobot(Gamepad gamepad1) {

        float modifiedYValue = -gamepad1.right_stick_y; //This is because the phone was receiving y values that were flipped from all controllers, resulting in backwards driving behavior
        positionalMovementFLPower = scaleInput(Range.clip((modifiedYValue + gamepad1.right_stick_x), -1, 1));
        positionalMovementFRPower = scaleInput(Range.clip((modifiedYValue - gamepad1.right_stick_x), -1, 1));
        positionalMovementRRPower = scaleInput(Range.clip((modifiedYValue + gamepad1.right_stick_x), -1, 1));
        positionalMovementRLPower = scaleInput(Range.clip((modifiedYValue - gamepad1.right_stick_x), -1, 1));
    }

    public void generalTelemetry(OpMode caller) {


        float negativePower = -(scaleInput(caller.gamepad1.left_trigger))/2;
        float positivePower = (scaleInput(caller.gamepad1.right_trigger))/2;
        float netPower = positivePower + negativePower;

        Telemetry telemetry = caller.telemetry;
        telemetry.clear();
        telemetry.addData("pressed? ", relicResetPressed);
        telemetry.addData("state", robot.relicLiftTouchSensor.getState());
        telemetry.addData("relic position", robot.relicLiftMotor.getCurrentPosition());
        telemetry.addData("neg ", negativePower);
        telemetry.addData("pos", positivePower);
        telemetry.addData("net", netPower);
        telemetry.addData("power ", robot.relicLiftMotor.getPower());
        telemetry.update();
    }



    public void armServos(Gamepad gamepad2, Telemetry telemetry) {

        if (gamepad2.x) {
            telemetry.addLine("Opening");
            telemetry.update();
            cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Mid, robot);
        }

        if (gamepad2.a) {

            telemetry.addLine("Mid Way");
            telemetry.update();
            cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Mid, robot);

        }
        if (gamepad2.b) {

            telemetry.addLine("Closing");
            telemetry.update();
            cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Close, robot);
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

    public void setLiftMotorPower(Gamepad gamepad2) {

        float rightPower = gamepad2.right_trigger;
        float leftPower = gamepad2.left_trigger;
        float netPower = rightPower - leftPower;
        boolean exceedingEncoderLimit = false;
        boolean belowEncoderLimit = false;

        if (robot.liftMotor.getCurrentPosition() >= 0) {

            if (robot.liftMotor.getCurrentPosition() > liftMotorFluidMaximum) {
                exceedingEncoderLimit = true;
            }
        } else {

            if (robot.liftMotor.getCurrentPosition() < liftMotorFluidMinimum) {
                belowEncoderLimit = true;
            }
        }

        if(!gamepad2.y && robot.liftMotor.getCurrentPosition() > liftMotorFluidMaximum) {

            robot.liftMotor.setPower(Range.clip(scaleInput((double)(-leftPower)), -1, 1));
        } else if (!gamepad2.y && robot.liftMotor.getCurrentPosition() < liftMotorFluidMinimum) {

            robot.liftMotor.setPower(Range.clip(scaleInput((double)(rightPower)), -1, 1));
        } else {

            robot.liftMotor.setPower(Range.clip(scaleInput((double)netPower), -1, 1));
        }
    }


    public void setRelicLiftPower(Gamepad gamepad1, OpMode caller) {

        float negativePower = -(scaleInput(gamepad1.left_trigger))/2;
        float positivePower = (scaleInput(gamepad1.right_trigger))/2;
        float netPower = positivePower + negativePower;

        if (!robot.relicLiftTouchSensor.getState() == true) {

            relicResetPressed = true;
        } else {

            relicResetPressed = false;
        }

        if (robot.relicLiftMotor.getCurrentPosition() > relicFluidMaximum) {

            caller.telemetry.addLine("RELIC LIFT ENCODER MAXIMUM REACHED");
            caller.telemetry.update();
        }

        if (relicResetPressed && !gamepad1.y) {

            robot.relicLiftMotor.setPower(positivePower);
            relicFluidMaximum = robot.relicLiftMotor.getCurrentPosition() + RELIC_MOTOR_MAXIMUM_POSITION;
        } else {

            robot.relicLiftMotor.setPower(netPower);
        }
    }

    public void manipulateRelicGrabber(Gamepad gamepad1){
        if (gamepad1.x){
            robot.relicGrabberServo.setPosition(RELIC_GRABBER_CLOSED);
        }
        if (gamepad1.y) {
            robot.relicGrabberServo.setPosition(RELIC_GRABBER_CLOSED + 0.1);
        }
        if (gamepad1.b){
            robot.relicGrabberServo.setPosition(RELIC_GRABBER_OPEN);
        }
    }

    public void endServoReset() {

        //robot.blockGrabberServo.setPosition(BLOCK_GRABBER_OPEN);
        cl.manipulateBlockGrabberPosition(CommonLibrary.Grabber.Mid, robot);
        //robot.rightArmServo.setPosition(RIGHT_ARM_OPEN);
        robot.relicGrabberServo.setPosition(RELIC_GRABBER_OPEN);
    }

    public void rotateGrabber(Gamepad gamepad1, Telemetry telemetry){
        if (gamepad1.dpad_up){
            robot.relicRotateServo.setPosition(RELIC_ROTATE_UP);
            telemetry.addLine("dpad up has been pressed");
            telemetry.update();
        }
        if (gamepad1.dpad_down){
            robot.relicRotateServo.setPosition(RELIC_ROTATE_DOWN);
            telemetry.addLine("dpad down has been pressed");
            telemetry.update();
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

    private static float scaleInput(float dVal)  {
        /**
         * Converts raw input into values that can be used as power arguments for motors and servos
         */

        float[] scaleArray = { 0.0f, 0.05f, 0.09f, 0.10f, 0.12f, 0.15f, 0.18f, 0.24f,
                0.30f, 0.36f, 0.43f, 0.50f, 0.60f, 0.72f, 0.85f, 1.00f, 1.00f };

        int index = (int) (dVal * 16.0);

        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        float dScale;

        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }


    public void paintRollersSpin(Gamepad gamepad2, Telemetry telemetry) {
        double spinStart = System.nanoTime();
        if(gamepad2.dpad_down) {
            telemetry.addLine("Suck in");
            telemetry.update();
            robot.leftRoller.setPower(1);
            robot.rightRoller.setPower(-1);
        } else if(gamepad2.dpad_up) {
            telemetry.addLine("Push out");
            telemetry.update();
            robot.leftRoller.setPower(-1);
            robot.rightRoller.setPower(1);
        } else {
            robot.leftRoller.setPower(0);
            robot.rightRoller.setPower(0);
        }
    }
}
