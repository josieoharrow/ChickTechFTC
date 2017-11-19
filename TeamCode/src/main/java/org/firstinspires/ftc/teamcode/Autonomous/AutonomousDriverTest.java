package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.pictoDecipher;
//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.initial;


/**
 * Created by Robotics on 8/27/2017.
 */


@Autonomous(name = "Test Autonomous")
//@Disabled
public class AutonomousDriverTest extends LinearOpMode {

    boolean runLinearCode = true;
    int teamColor = 0;
    String vuforiaPosition = "unknown";

    @Override
    public void runOpMode () throws InterruptedException {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap);
        double SCALE_FACTOR = 255;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        waitForStart();

        while (opModeIsActive()) {
            Color.RGBToHSV((int) (al.robot.colorSensorREV.red() * SCALE_FACTOR),
                    (int) (al.robot.colorSensorREV.green() * SCALE_FACTOR),
                    (int) (al.robot.colorSensorREV.blue() * SCALE_FACTOR),
                    hsvValues);

            if (runLinearCode) {

                teamColor = al.setTeamColor();
                al.decipherJewelAndKnockOff(telemetry, this);
                al.robot.jewelActuatorServo.setPosition(0.3);
                vuforiaPosition = al.pictoDecipher(telemetry, this);
                // al.driveAtAngle(18, 135, telemetry, this);// switch to unit circle
                al.closeArms();
                if (teamColor == 1) {
                    al.driveAtAngle(25, 90, telemetry, this);
                } else {
                    al.driveAtAngle(25, 30, telemetry, this);
                }
                al.driveToVuforiaPositionFromTheLeft(telemetry, this, vuforiaPosition);
                al.driveAtAngle(4, 90, telemetry, this);
                al.openArms();
            }

            runLinearCode = false;
        }
    }
    }
