package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.RobotHardware;

/**
 * Created by Robotics on 8/27/2017.
 */
public class TeleOpLibrary {

    RobotHardware robot   = new RobotHardware();

    void declareRobot(RobotHardware robotSent) {

        robot = robotSent;
    }
}
