package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by Robotics on 8/27/2017.
 */
@Autonomous(name = "Main Autonomous")
//@Disabled
public class AutonomousDriver extends LinearOpMode {

    boolean runLinearCode = true;

    @Override
    public void runOpMode() {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            if (runLinearCode) {
                telemetry.addLine("First drive");
                telemetry.update();
                al.driveAtAngle(18, 45, telemetry, this);

                telemetry.addLine("Vuforia");
                telemetry.update();
                al.pictoDecipher(telemetry, this);
                telemetry.addLine("2nd drive");
                telemetry.update();
                al.driveAtAngle(12, 70, telemetry, this);
            }

            runLinearCode = false;
        }
    }
}