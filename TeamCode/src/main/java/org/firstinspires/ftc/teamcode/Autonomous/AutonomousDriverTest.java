package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.RobotHardware;
//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.pictoDecipher;
//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.initial;


/**
 * Created by Robotics on 8/27/2017.
 */


@Autonomous(name = "Test Autonomous")
@Disabled
public class AutonomousDriverTest extends LinearOpMode {

    boolean runLinearCode = true;

    @Override
    public void runOpMode () throws InterruptedException {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            if (runLinearCode) {
                telemetry.addLine("First drive");
                telemetry.update();
                al.driveAtAngle(18, -45, telemetry, this);

                telemetry.addLine("Vuforia");
                telemetry.update();
                al.pictoDecipher(telemetry, this);
                telemetry.addLine("2nd drive");
                telemetry.update();
                al.driveAtAngle(12, -70, telemetry, this);

            }

            runLinearCode = false;
        }
    }
            /* al.MotorEncoderTest(telemetry);

        robot.init(hardwareMap);
        CommonLibrary cl = new CommonLibrary();
        cl.declareRobot(robot);*/
}
