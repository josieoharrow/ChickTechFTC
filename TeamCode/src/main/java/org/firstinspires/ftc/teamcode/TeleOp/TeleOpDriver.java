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
    CommonLibrary cl;
    Boolean gyroInitialized = false;

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


        if (!gyroInitialized) {

            Thread t1 = new Thread(new Runnable() {
                public void run() {
                    tol.initGyro();
                }
            });
            t1.start();
            gyroInitialized = true;
        }

        tol.translateRightStickToSlidingRelativeToField(gamepad1, telemetry);
        tol.translateLeftStickToRotation(gamepad1);
        tol.setDrivingMotorPowers(gamepad1, telemetry);
        tol.armServos(gamepad1, gamepad2, telemetry);
        tol.resetLiftMotorEncoderBasedOnTouchSensorActivation(telemetry);
        tol.setDrivingMotorPowers(gamepad1, telemetry);
        tol.setLiftMotorPower(gamepad2, telemetry);
        tol.resetLiftMotorEncoderBasedOnTouchSensorActivation(telemetry);
        tol.generalTelemetry(gamepad1, gamepad2, telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
