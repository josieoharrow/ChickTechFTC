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
    OpMode op;
    Boolean gyroInitialized = false;
    Boolean liftLowered = false;

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

        op = this;

        if (!gyroInitialized) {

            Thread t1 = new Thread(new Runnable() {
                public void run() {
                    tol.initGyro();
                }
            });
            t1.start();
            gyroInitialized = true;
        }

        if (!liftLowered) {

            Thread t2 = new Thread(new Runnable() {
                public void run() {
                    tol.lowerLift();
                }
            });
            t2.start();
            liftLowered = true;
        } else {
            
            tol.setLiftMotorPower(gamepad2);
        }

        tol.setDrivingMotorPowers(gamepad1, telemetry);
        tol.armServos(gamepad2, telemetry);
        tol.resetLiftMotorEncoderBasedOnTouchSensorActivation(telemetry);
        tol.setDrivingMotorPowers(gamepad1, telemetry);
        tol.generalTelemetry(this);
        tol.manipulateGrabber(gamepad1);
        tol.setRelicLiftPower(gamepad1, this);
        tol.rotateGrabber(gamepad1, telemetry);

        /*if (op.getRuntime() >= 122) {

            tol.endServoReset();
        }*/
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        tol.endServoReset();//test

    }
}
