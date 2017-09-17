package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

/**
 * Created by Robotics on 8/27/2017.
 */
@TeleOp(name = "Test TeleOp")
//@Disabled
public class TeleOpDriverTest extends OpMode {

    RobotHardware robot   = new RobotHardware();

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        CommonLibrary cl = new CommonLibrary();
        cl.declareRobot(robot);
        TeleOpLibrary tol = new TeleOpLibrary();
        tol.declareRobot(robot);
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

        if (gamepad1.y)
            robot.frontMotor.setPower(1);
        else
            robot.frontMotor.setPower(0);

        if (gamepad1.x)
            robot.leftMotor.setPower(1);
        else
            robot.leftMotor.setPower(0);

        if (gamepad1.a)
            robot.backMotor.setPower(1);
        else
            robot.backMotor.setPower(0);

        if (gamepad1.b)
            robot.rightMotor.setPower(1);
        else
            robot.rightMotor.setPower(0);

        if (gamepad1.dpad_up)
            robot.frontMotor.setPower(-1);
        else
            robot.frontMotor.setPower(0);

        if (gamepad1.dpad_left)
            robot.leftMotor.setPower(-1);
        else
            robot.leftMotor.setPower(0);

        if (gamepad1.dpad_down)
            robot.backMotor.setPower(-1);
        else
            robot.backMotor.setPower(0);

        if (gamepad1.dpad_right)
            robot.rightMotor.setPower(-1);
        else
            robot.rightMotor.setPower(0);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
