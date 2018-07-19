package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;

/**
 * Created by Robotics on 8/27/2017.
 */

@TeleOp(name = "TeleOp")
//@Disabled
public class TeleOpDriver extends OpMode {

    TeleOpLibrary tol;

    @Override
    public void init() {
        /* Initialize the hardware variables.
        */
        tol = new TeleOpLibrary();
        tol.init(this);
        telemetry.addLine("Initializing complete.");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        tol.updateTelemetryWithText("Running TeleOp");
    }
}
