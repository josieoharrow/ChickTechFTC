package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.pictoDecipher;
//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.initial;


/**
 * Created by Robotics on 8/27/2017.
 */


@Autonomous(name = "Autonomous with MR Sensor")
//@Disabled
public class AutonomousDriverTest extends LinearOpMode {

    boolean runLinearCode = true;
    boolean setFirstBlockInFarLeftColumn = false;
    String vuforiaPosition = "unknown";

    static float JEWEL_ACTUATOR_UP = 0.2f;
    RobotHardware robot;
    int columnPlacement;
    @Override
    public void runOpMode() throws InterruptedException {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap, telemetry, gamepad1, this);
        CommonLibrary cl = new CommonLibrary();
        cl.init(hardwareMap);
        Boolean ran = false;
        telemetry.addLine("Hey now, Hey now.\n(You can start Autonomous now)");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {


        }

    }

}
