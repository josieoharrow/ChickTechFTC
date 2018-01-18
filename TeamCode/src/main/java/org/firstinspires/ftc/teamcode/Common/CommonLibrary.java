package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        robot.init(hardwareMap);
    }


    public void resetLiftMotorEncoder() {

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (robot.liftMotor.isBusy()) {
        }
    }


    public void wait(double milliseconds, LinearOpMode caller) {

        long startTime =  System.currentTimeMillis();

        long currentTime = System.currentTimeMillis();

        while ((currentTime - startTime) < milliseconds && !caller.isStopRequested()) {

            currentTime = System.currentTimeMillis();
        }
    }
}
