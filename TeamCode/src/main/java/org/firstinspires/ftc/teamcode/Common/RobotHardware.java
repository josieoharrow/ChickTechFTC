package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Robotics on 8/27/2017.
 */
public class RobotHardware {

    /* Public OpMode members. */
    public DcMotor leftMotor;
    /* local OpMode members. */
    HardwareMap hardwareMap;

    /* Constructor */
    public RobotHardware(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;

        // Define and Initialize
        leftMotor   = hardwareMap.dcMotor.get("left_drive");
    }
}
