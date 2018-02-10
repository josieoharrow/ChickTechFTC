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

/**
 * Created by Robotics on 8/27/2017.
 */
public class RobotHardware {

    //CommonLibrary cl;

    /* Public OpMode members. */
    public DcMotor frontLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor liftMotor;
    public DcMotor relicLiftMotor;
    public Servo jewelActuatorServo;
    public Servo relicGrabberServo;
    public Servo relicRotateServo;
    public Servo leftGrabber;
    public Servo rightGrabber;
    public DcMotor leftRoller;
    public DcMotor rightRoller;
    public DigitalChannel liftMotorTouchSensor;
    public DigitalChannel relicLiftTouchSensor;
    public LynxI2cColorRangeSensor leftSensorDistance;
    public LynxI2cColorRangeSensor rightSensorDistance;
    public ModernRoboticsI2cRangeSensor mrRangeSensor;
    public ColorSensor colorSensorREV;
    public BNO055IMU imu;

    static float JEWEL_ACTUATOR_UP = 0.15f;
    static float RELIC_GRABBER_DOWN = .1f;
    static float RELIC_ROTATE_DOWN = 1f;

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
        relicGrabberServo = hardwareMap.servo.get("relic grabber servo");
        relicRotateServo = hardwareMap.servo.get("relic rotate servo");
        leftGrabber = hardwareMap.servo.get("left grabber");
        rightGrabber = hardwareMap.servo.get("right grabber");
        liftMotorTouchSensor = hardwareMap.digitalChannel.get("lift motor touch sensor");
        relicLiftTouchSensor = hardwareMap.digitalChannel.get("relic lift touch sensor");
        //leftRoller = hardwareMap.dcMotor.get("left roller");
        //rightRoller = hardwareMap.dcMotor.get("right roller");
        //leftSensorDistance = hardwareMap.get(LynxI2cColorRangeSensor.class, "left ds");
        //rightSensorDistance = hardwareMap.get(LynxI2cColorRangeSensor.class, "right ds");
        mrRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr range sensor");
        colorSensorREV.setI2cAddress(I2cAddr.create7bit(0x39));
        liftMotorTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
        relicGrabberServo.setPosition(RELIC_GRABBER_DOWN);
        relicRotateServo.setPosition(RELIC_ROTATE_DOWN);
        leftGrabber.setPosition(CommonLibrary.LEFT_GRABBER_OPEN);
        rightGrabber.setPosition(CommonLibrary.RIGHT_GRABBER_OPEN);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (rearLeftMotor.isBusy()) {

        }
    }

    public void initGyro() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        try {
            imu.initialize(parameters);
            while(!imu.isGyroCalibrated()){

            } } catch (Exception e) {
        }
    }
}
