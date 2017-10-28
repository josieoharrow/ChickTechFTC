package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Servo    leftClaw    = null;
    public Servo    rightClaw    = null;
    public Servo    middleClaw    = null;

    public Servo leftArm;
    //public Servo rightArm;

    public BNO055IMU imu;

    /* local OpMode members. */
    HardwareMap hardwareMap;

    /* Constructor */
    public RobotHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        // Define and Initialize
        hardwareMap = hwMap;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeftMotor = hardwareMap.dcMotor.get("front left motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear right motor");
        frontRightMotor = hardwareMap.dcMotor.get("front right motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear left motor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm = hardwareMap.servo.get("left arm");
        //rightArm = hardwareMap.servo.get("right arm");
        leftArm.setPosition(0);
        //rightArm.setPosition(0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while(!imu.isGyroCalibrated()){
        }
    }
}
