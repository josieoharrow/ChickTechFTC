package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.RobotHardware;

//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.pictoDecipher;
//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.initial;


/**
 * Created by Robotics on 8/27/2017.
 */


@Autonomous(name = "Test Autonomous")
//@Disabled
public class AutonomousDriverTest extends LinearOpMode {

    boolean runLinearCode = true;
    String vuforiaPosition = "unknown";
    RobotHardware robot;

    @Override
    public void runOpMode () throws InterruptedException {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap, telemetry, gamepad1, this);
        robot = new RobotHardware();
        Boolean ran = false;
        robot.init(hardwareMap);
        waitForStart();


        while (opModeIsActive()) {

            if (!ran) {
                telemetry.addLine("HERE");
                telemetry.update();
                al.blockFollow(this);
                ran = true;
            }
            telemetry.addLine("out of method");
            telemetry.update();
        }
    }
    }
