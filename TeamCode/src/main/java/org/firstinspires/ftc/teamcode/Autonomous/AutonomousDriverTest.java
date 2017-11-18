package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

                telemetry.addData("team color = ", al.robot.isRed);
                al.decipherJewelAndKnockOff(telemetry, this);
                //al.driveAtAngle(18, 135, telemetry, this);// switch to unit circle

/*
                telemetry.addLine("Vuforia");
                telemetry.update();
                al.pictoDecipher(telemetry, this);
                telemetry.addLine("2nd drive");
                telemetry.update();*/

            }

            runLinearCode = false;
       /* }
    }

            /* al.MotorEncoderTest(telemetry);

        robot.init(hardwareMap);
        CommonLibrary cl = new CommonLibrary();
        cl.declareRobot(robot);
            }*/
        }
    }
}

