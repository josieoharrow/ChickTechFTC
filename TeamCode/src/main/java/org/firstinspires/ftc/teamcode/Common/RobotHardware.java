package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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
    public DcMotor relicLiftMotor;
    public Servo leftArmServo;
    public Servo rightArmServo;
    public Servo rampServo;
    public Servo jewelActuatorServo;
    public Servo relicGrabberServo;
    public Servo relicRotateServo;
    public DigitalChannel liftMotorTouchSensor;
    public AnalogInput relicLiftTouchSensor;
    public OpticalDistanceSensor leftODS;
    public OpticalDistanceSensor rightODS;

    //public ColorSensor colorSensorMR;
    public ColorSensor colorSensorREV;
    public BNO055IMU imu;

    static float JEWEL_ACTUATOR_UP = 0.2f;

    /* local OpMode members. */
    HardwareMap hardwareMap;

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
        relicLiftMotor = hardwareMap.dcMotor.get("relic lift motor");
        colorSensorREV = hardwareMap.get(ColorSensor.class, "jewel color sensor");
        jewelActuatorServo = hardwareMap.servo.get("jewel actuator");
        leftArmServo = hardwareMap.servo.get("left arm");
        rightArmServo = hardwareMap.servo.get("right arm");
        relicGrabberServo = hardwareMap.servo.get("relic grabber servo");
        relicRotateServo = hardwareMap.servo.get("relic rotate servo");
        liftMotorTouchSensor = hardwareMap.digitalChannel.get("lift motor touch sensor");
        relicLiftTouchSensor = hardwareMap.analogInput.get("relic lift touch sensor");//AI get voltage if necessary
        //leftODS = hardwareMap.opticalDistanceSensor.get("left ods");
        //rightODS = hardwareMap.opticalDistanceSensor.get("right ods");
        liftMotorTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        //rampServo = hardwareMap.servo.get("ramp servo");
        jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);

        leftArmServo.setPosition(0.07);
        rightArmServo.setPosition(0.93);
        relicGrabberServo.setPosition(.9);
        relicRotateServo.setPosition(1);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //relicLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initGyro() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        try {
            imu.initialize(parameters);
            while(!imu.isGyroCalibrated()){

            } } catch (Exception e) {
        }
    }
}
