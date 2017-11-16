package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

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


    @Override
    public void runOpMode () throws InterruptedException {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            if (runLinearCode) {


                al.turnToAngleWithPID(90, 0.0042,  0.0002, 0, telemetry, this);
                /*Thread.sleep(100);
                al.turnToAngleWithPID(-45, 0.0042, 0.0002, 0, telemetry, this);
                Thread.sleep(100);
                al.turnToAngleWithPID(180, 0.0042, 0.0002, 0, telemetry, this);
                Thread.sleep(100);
                al.turnToAngleWithPID(315, 0.0042, 0.0002, 0, telemetry, this);
                al.turnToAngleWithPID(-90, 0.0005, 0.0002, 0, telemetry, this);
                /*telemetry.addData("team color = ", al.robot.isRed);
                telemetry.update();

                al.driveAtAngle(18, 315, telemetry, this);

                telemetry.addData("team color = ", al.robot.isRed);
                al.setPosition(telemetry);
                //al.driveAtAngle(18, 315, telemetry, this);
                telemetry.addLine("Vuforia");
                telemetry.update();
                al.pictoDecipher(telemetry, this);
                telemetry.addLine("2nd drive");
                telemetry.update();*/

            }

            runLinearCode = false;
        }
    }

            /* al.MotorEncoderTest(telemetry);

        robot.init(hardwareMap);
        CommonLibrary cl = new CommonLibrary();
        cl.declareRobot(robot);*/
}

