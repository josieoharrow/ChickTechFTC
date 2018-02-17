package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

/**
 * Created by Robotics on 8/27/2017.
 */
public class CommonLibrary {

    public enum Grabber {
        Open, Mid, Close
    }

    HardwareMap hardwareMap;

    static final double RIGHT_GRABBER_OPEN = 0.98;
    static final double RIGHT_GRABBER_MID = 0.775;//0.5
    static final double RIGHT_GRABBER_CLOSED = 0.47; //0.63
    static final double LEFT_GRABBER_OPEN = 0.02;
    static final double LEFT_GRABBER_MID = 0.225; //0.46
    static final double LEFT_GRABBER_CLOSED = 0.45; //0.41

    public void init(HardwareMap hardwareMapSent) {

        hardwareMap = hardwareMapSent;
    }



    public void wait(double milliseconds, LinearOpMode caller) {

        long startTime =  System.currentTimeMillis();

        long currentTime = System.currentTimeMillis();

        while ((currentTime - startTime) < milliseconds && !caller.isStopRequested()) {

            currentTime = System.currentTimeMillis();
        }
    }

    public void manipulateBlockGrabberPosition(Grabber position, RobotHardware robot){

        switch (position){
            case Open:
                robot.leftGrabber.setPosition(LEFT_GRABBER_OPEN);
                robot.rightGrabber.setPosition(RIGHT_GRABBER_OPEN);
                break;
            case Mid:
                robot.leftGrabber.setPosition(LEFT_GRABBER_MID);
                robot.rightGrabber.setPosition(RIGHT_GRABBER_MID);
                break;
            case Close:
                robot.leftGrabber.setPosition(LEFT_GRABBER_CLOSED);
                robot.rightGrabber.setPosition(RIGHT_GRABBER_CLOSED);
        }
    }

    public void servoControllerEnabling(RobotHardware robot) {
        robot.sc.pwmEnable();
    }

    public void servoControllerDisabling(RobotHardware robot){
        robot.sc.pwmDisable();
    }
}
