package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;


/**
 * Created by Robotics on 8/27/2017.
 */
@Autonomous(name = "Autonomous Test")
//@Disabled

public class AutonomousDriverTest extends LinearOpMode {

    RobotHardware robot;
    boolean runLinearCode = true;
    String vuforiaPosition = "unknown";

    @Override
    public void runOpMode() {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap, telemetry, gamepad1, this);
        CommonLibrary cl = new CommonLibrary();
        cl.init(hardwareMap);
        telemetry.addLine("ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("MR ", robot.mrRangeSensor.getI2cAddress());
            telemetry.addData("MR ", robot.mrRangeSensor.rawUltrasonic());
            telemetry.addData("MR ", robot.mrRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}