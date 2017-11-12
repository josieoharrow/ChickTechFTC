package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Robotics on 8/27/2017.
 */
public class CommonLibrary {

    RobotHardware robot;
    HardwareMap hardwareMap;


    public void init(HardwareMap hardwareMapSent) {

        hardwareMap = hardwareMapSent;
        robot = new RobotHardware();
        setTeamColor();
        robot.init(hardwareMap);
    }

    public void setTeamColor() {

        if (robot.colorSensorMR.red() > robot.colorSensorMR.blue()){
            robot.isRed = 1;
        }
        else {
            robot.isRed = 0;
        }
    }
}
