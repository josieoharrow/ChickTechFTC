package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;

/**
 * Created by Robotics on 8/27/2017.
 */
@TeleOp(name = "Test TeleOp")
//@Disabled
public class TeleOpDriverTest extends OpMode {

    TeleOpLibrary tol;
    CommonLibrary cl;
    Boolean gyroInitialized = false;
    Boolean g1left = false;
    Boolean g1right = false;
    Boolean g1up = false;
    Boolean g1down = false;
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        telemetry.addLine("Initializing. please wait.");
        telemetry.update();
        cl = new CommonLibrary();
        telemetry.addLine("Initializing CommonLibrary. Please wait.");
        telemetry.update();
        cl.init(hardwareMap);
        tol = new TeleOpLibrary();
        telemetry.addLine("Initializing TeleOpLibrary. Please wait.");
        telemetry.update();
        tol.init(hardwareMap);
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



       if (gamepad1.dpad_left && tol.robot.relicRotateServo.getPosition() != 1) {

            g1left = true;
        } else if (g1left) {
            tol.setServoPosition(tol.robot.relicRotateServo, (tol.robot.relicRotateServo.getPosition() + 0.1));
            g1left = false;
        }
        if (gamepad1.dpad_right && tol.robot.relicRotateServo.getPosition() != 0) {

            g1right = true;
        } else if (g1right) {
            tol.setServoPosition(tol.robot.relicRotateServo, (tol.robot.relicRotateServo.getPosition() - 0.1));
            g1right = false;
        }

        if (gamepad1.dpad_up && tol.robot.relicGrabberServo.getPosition() != 1) {
            g1up = true;
        } else if (g1up) {
            tol.setServoPosition(tol.robot.relicGrabberServo, (tol.robot.relicGrabberServo.getPosition() + 0.1));

            g1up = false;
        }

        if (gamepad1.dpad_down && tol.robot.relicGrabberServo.getPosition() != 0) {
            g1down = true;
        } else if (g1down) {

            tol.setServoPosition(tol.robot.relicGrabberServo, (tol.robot.relicGrabberServo.getPosition() - 0.1));

            g1down = false;
        }

       // tol.setServoPosition(tol.robot.relicGrabberServo, (tol.robot.relicGrabberServo.getPosition() + 0.1));

        telemetry.addData("grabber position ", tol.robot.relicGrabberServo.getPosition());
        telemetry.addData("ROTATE position ", tol.robot.relicRotateServo.getPosition());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
