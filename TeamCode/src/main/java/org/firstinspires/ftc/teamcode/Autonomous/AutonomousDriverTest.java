package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;


/**
 * Created by Robotics on 8/27/2017.
 */
@Autonomous(name = "Test Autonomous")
//@Disabled
public class AutonomousDriverTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    boolean runLinearCode = true;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        CommonLibrary cl = new CommonLibrary();
        cl.declareRobot(robot);
        AutonomousLibrary al = new AutonomousLibrary();
        al.declareRobot(robot);
        al.init();
        waitForStart();

        while (opModeIsActive()) {
           // al.MotorEncoderTest(telemetry);
            //al.declareRobot(robot);
            if (runLinearCode) {

                al.driveAtAngle(10, 45, telemetry, this);

                al.driveAtAngle(10, 180, telemetry, this);

                al.driveAtAngle(10, 270, telemetry, this);
            }
            runLinearCode = false;
        }
    }
}
