package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Concept: Ramp Motor Speed", group = "Concept")
//@Disabled
public class FirstDriver_Test extends LinearOpMode {

    // Define class members
    DcMotor leftMotor;
    double  power   = 1;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("left motor");

        // Wait for the start button
        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {



            leftMotor.setPower(power);
        }

        // Turn off motor and signal done;
        //leftMotor.setPower(0);

    }
}
