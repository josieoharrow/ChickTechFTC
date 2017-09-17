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

    RobotHardware robot   = new RobotHardware();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        CommonLibrary cl = new CommonLibrary();
        cl.declareRobot(robot);
        AutonomousLibrary al = new AutonomousLibrary();
        al.declareRobot(robot);
        waitForStart();

        while (opModeIsActive()) {

            break;
            //last step
        }
    }
}
