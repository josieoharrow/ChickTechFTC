package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by Chicktech on 7/19/2018.
 */
public class RobotHardware {

    /* Public OpMode members. */
    public DcMotor rightMotor;
    public DcMotor leftMotor;
    public Servo servoOne;
    public Servo servoTwo;
    public OpticalDistanceSensor odsSensor;
    public ModernRoboticsI2cGyro gyro;
    public TouchSensor touchSensor;

    /* local OpMode members. */
    HardwareMap hardwareMap;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        // Define and Initialize
        hardwareMap = hwMap;
        frontLeftMotor = hardwareMap.dcMotor.get("leftMotor");
        rearRightMotor = hardwareMap.dcMotor.get("rightMotor");

        servoOne = hardwareMap.servo.get("servoOne");
        servoTwo = hardwareMap.servo.get("servoTwo");

        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        gyro = hardwareMap.modernRoboticsI2cGyro.get("gyro");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (leftMotor.isBusy()) {
        }
    }
}
