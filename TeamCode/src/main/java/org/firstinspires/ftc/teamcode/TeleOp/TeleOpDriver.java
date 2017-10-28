package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PWMOutput;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;
import com.qualcomm.robotcore.hardware.PWMOutputImpl;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;

/**
 * Created by Robotics on 8/27/2017.
 */
@TeleOp(name = "Main TeleOp")
//@Disabled
public class TeleOpDriver extends OpMode {

    RobotHardware robot   = new RobotHardware();

    double five;

    public double usPulseUpper = 2500;
    double usFrame;
    public double usPulseLower = 500;


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




        while(gamepad2.dpad_up)
            robot.leftClaw.setPosition(1);
        while (gamepad2.dpad_down)
            robot.leftClaw.setPosition(0.5);

        if(gamepad2.dpad_left)
            robot.rightClaw.setPosition(1);

        else if(gamepad2.dpad_right)
            robot.rightClaw.setPosition(0);

        if(gamepad2.x) {
            robot.middleClaw.setPosition(1);
        }

        else if(gamepad2.b) {
            robot.middleClaw.setPosition(0);
            telemetry.addData("", "");
            telemetry.update();
        }




    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
