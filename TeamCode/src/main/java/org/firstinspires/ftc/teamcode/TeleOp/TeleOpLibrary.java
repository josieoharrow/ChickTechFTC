package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Common.RobotHardware;

/**
 * Created by Robotics on 8/27/2017.
 */
public class TeleOpLibrary {

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
