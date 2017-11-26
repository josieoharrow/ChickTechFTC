package org.firstinspires.ftc.teamcode.Autonomous;

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
        al.init(hardwareMap, telemetry, gamepad1, this);
        double SCALE_FACTOR = 255;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        waitForStart();

        while (opModeIsActive()) {

            if (runLinearCode) {

                al.driveAtAngle(192, 90, telemetry, this);

                /*telemetry.addLine("I am here");
                telemetry.update();

            if (runLinearCode) {

                al.turnToAngleWithPID(90, telemetry, this);
                Thread.sleep(1000);
                al.turnToAngleWithPID(-405, telemetry, this);
                Thread.sleep(1000);
                al.turnToAngleWithPID(3, telemetry, this);
                Thread.sleep(1000);
                al.turnToAngleWithPID(180, telemetry, this);
                /*teamColor = al.setTeamColor();
              //  teamColor = al.setTeamColor(telemetry);
                al.decipherJewelAndKnockOff(telemetry, this);
                al.robot.jewelActuatorServo.setPosition(0.3);
                vuforiaPosition = al.pictoDecipher(telemetry, this);
                telemetry.addData("vuforia ", vuforiaPosition);
                telemetry.update();
                al.closeArms();

                if (al.teamColorAndPosition == 1) {
                    //red team corner balance board
                    al.turnToAngleWithPID(-90, telemetry, this);
                    al.driveAtAngle(24, 180, telemetry, this);
                    al.driveAtAngle(6, 90, telemetry, this);
                    al.driveToVuforiaPositionFromTheRight(telemetry, this, vuforiaPosition);
                } else if (al.teamColorAndPosition == 2) {
                    //red ream center balance board
                    al.turnToAngleWithPID(180, telemetry, this);
                    al.driveAtAngle(36, 90, telemetry, this);
                    al.driveToVuforiaPositionFromTheRight(telemetry, this, vuforiaPosition);
                } else if (al.teamColorAndPosition == 3) {
                    //blue team corner balance board

                    al.turnToAngleWithPID(90, telemetry, this);
                    al.driveAtAngle(24, 0, telemetry, this);
                    al.driveAtAngle(6, 90, telemetry, this);
                    al.driveToVuforiaPositionFromTheLeft(telemetry, this, vuforiaPosition);

                } else if (al.teamColorAndPosition == 4) {

                    //blue team center balance board
                    al.driveAtAngle(36, 90, telemetry, this);
                    al.driveToVuforiaPositionFromTheLeft(telemetry, this, vuforiaPosition);

                } else {
                    telemetry.addLine("I don't know where I am who I am what's going oNNNN HELP ME");
                    telemetry.update();
                }


                al.driveAtAngle(4, 90, telemetry, this);
                al.openArms();*/

                al.openArms();
            }

            runLinearCode = false;
        }
    }
    }
