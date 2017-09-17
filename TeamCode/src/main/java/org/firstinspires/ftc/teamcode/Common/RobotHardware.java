package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Robotics on 8/27/2017.
 */
public class RobotHardware {

    /* Public OpMode members. */
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor frontMotor;
    public DcMotor backMotor;

    /* local OpMode members. */
    HardwareMap hardwareMap;

    /* Constructor */
    public RobotHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        // Define and Initialize
        hardwareMap = hwMap;
        leftMotor   = hardwareMap.dcMotor.get("left_drive");
        rightMotor  = hardwareMap.dcMotor.get("right_drive");
        frontMotor  = hardwareMap.dcMotor.get("front_drive");
        backMotor   = hardwareMap.dcMotor.get("back_drive");
    }
}
