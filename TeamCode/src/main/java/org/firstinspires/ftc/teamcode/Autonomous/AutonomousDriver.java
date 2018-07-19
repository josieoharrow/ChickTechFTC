package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;


/**
 * Created by Chicktech on 7/19/2018.
 */
@Autonomous(name = "Autonomous Driver")
//@Disabled

public class AutonomousDriver extends LinearOpMode {

    boolean runLinearCode = true;

    @Override
    public void runOpMode() {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(this);

        telemetry.addLine("Ready to start.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (runLinearCode) {

                al.updateTelemetryWithText("I am running code.");
            }

            runLinearCode = false;
        }
    }
}