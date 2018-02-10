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

    static final double RIGHT_GRABBER_OPEN = 1;
    static final double RIGHT_GRABBER_MID = 0.7;
    static final double RIGHT_GRABBER_CLOSED = 0.4;
    static final double LEFT_GRABBER_OPEN = 0;
    static final double LEFT_GRABBER_MID = 0.3;
    static final double LEFT_GRABBER_CLOSED = 0.6;

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
        //for arms? I think?
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
}
