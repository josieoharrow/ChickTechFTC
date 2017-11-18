package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 8/27/2017.
 */
public class RobotHardware {

    /* Public OpMode members. */
    public DcMotor frontLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor liftMotor;
    public Servo leftArmServo;
    public Servo rightArmServo;
    public Servo rampServo;

    public Servo jewelActuatorServo;

    public ColorSensor colorSensorMR;
    public ColorSensor colorSensorREV;
    public DigitalChannel positionTouchSensor;
    //public Servo rightArm;

    public BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hardwareMap;

    public int isRed = 3;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        // Define and Initialize
        hardwareMap = hwMap;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontLeftMotor = hardwareMap.dcMotor.get("front left motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear right motor");
        frontRightMotor = hardwareMap.dcMotor.get("front right motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear left motor");
        liftMotor = hardwareMap.dcMotor.get("lift motor");
        colorSensorMR = hardwareMap.get(ColorSensor.class, "ground color sensor");
        colorSensorREV = hardwareMap.get(ColorSensor.class, "jewel color sensor");
        //positionTouchSensor = hardwareMap.get(DigitalChannel.class, "position touch sensor");
        //positionTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        jewelActuatorServo = hardwareMap.servo.get("jewel actuator");
        leftArmServo = hardwareMap.servo.get("left arm");
        rightArmServo = hardwareMap.servo.get("right arm");
        //rampServo = hardwareMap.servo.get("ramp servo");
        jewelActuatorServo.setPosition(0.3);
        leftArmServo.setPosition(0);
        rightArmServo.setPosition(1);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);
        while(!imu.isGyroCalibrated()){

        }
    }
}
