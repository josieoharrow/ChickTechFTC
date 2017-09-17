package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Robotics on 8/27/2017.
 */
public class CommonLibrary {

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
}
