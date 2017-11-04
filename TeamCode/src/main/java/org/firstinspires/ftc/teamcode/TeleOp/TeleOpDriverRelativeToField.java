package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

/**
 * Created by Robotics on 8/27/2017.
 */
@TeleOp(name = "TeleOp Relative to Field")
//@Disabled
public class TeleOpDriverRelativeToField extends OpMode {

    RobotHardware robot   = new RobotHardware();
    TeleOpLibrary tol;
    CommonLibrary cl;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
        */
        robot.init(hardwareMap);
        cl = new CommonLibrary();
        cl.init(hardwareMap);
        tol = new TeleOpLibrary();
        tol.init(hardwareMap);
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

        tol.translateLeftStickToRotation(gamepad1);
        tol.telemetry(gamepad1, telemetry);
        tol.translateRightStickToSlidingRelativeToField(gamepad1);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
