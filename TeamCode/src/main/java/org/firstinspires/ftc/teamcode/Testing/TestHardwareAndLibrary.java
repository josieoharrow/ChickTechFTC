package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Robotics on 8/27/2017.
 */
public class TestHardwareAndLibrary {

    // Declare Hardware
    public DigitalChannel whiskerTouchRight;
    public DigitalChannel whiskerTouchLeft;

    //Declare Constants and Variables

    HardwareMap hardwareMap;
    public void init(HardwareMap hwMap) {

        //Define and Initialize hardware
        hardwareMap = hwMap;
        whiskerTouchRight = hardwareMap.digitalChannel.get("right whisker touch");
        whiskerTouchLeft = hardwareMap.digitalChannel.get("left whisker touch");

        //Init Motor and Positions
    }

    public void whiskerTouchTelemetry(Telemetry telemetry){

        if (whiskerTouchRight.getState() == true){
            telemetry.addLine("The right touch sensor is pressed");
            telemetry.addLine("I found the column!!!");
            telemetry.update();
        } else if (whiskerTouchLeft.getState() == true){
            telemetry.addLine("The left touch sensor is pressed");
            telemetry.addLine("I found the column!!!");
            telemetry.update();
        } else {
            telemetry.addLine("No touch, no column found");
            telemetry.update();
        }
    }


}
