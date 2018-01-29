package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Robotics on 8/27/2017.
 */
public class CommonLibrary {

    public enum Grabber {
        Open, Mid, Close
    }

    RobotHardware robot;
    HardwareMap hardwareMap;

    static final double RIGHT_GRABBER_OPEN = 0;
    static final double RIGHT_GRABBER_MID = 0.4;
    static final double RIGHT_GRABBER_CLOSED = 0.5;
    static final double LEFT_GRABBER_OPEN = 0.9;
    static final double LEFT_GRABBER_MID = 0.4;
    static final double LEFT_GRABBER_CLOSED = 0.3;

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

    /*public void manipulateGrabberPosition(int position){
        if (position == 0) {
            //close
            robot.leftGrabber.setPosition(LEFT_GRABBER_CLOSED);
            robot.rightGrabber.setPosition(RIGHT_GRABBER_CLOSED);
        } else if (position == 1){
            //mid
            robot.leftGrabber.setPosition(LEFT_GRABBER_MID);
            robot.rightGrabber.setPosition(RIGHT_GRABBER_MID);
        } else if (position == 2){
            //open
            robot.leftGrabber.setPosition(LEFT_GRABBER_OPEN);
            robot.rightGrabber.setPosition(RIGHT_GRABBER_OPEN);
        }
    }*/
    public void manipulateGrabberPosition(Grabber position){
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
