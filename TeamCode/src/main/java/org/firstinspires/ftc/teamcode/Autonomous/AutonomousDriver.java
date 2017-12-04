package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;


/**
 * Created by Robotics on 8/27/2017.
 */
@Autonomous(name = "Main Autonomous")
//@Disabled
public class AutonomousDriver extends LinearOpMode {

    boolean runLinearCode = true;
    int teamColor;
    String vuforiaPosition = "unknown";

    static float JEWEL_ACTUATOR_UP = 0.2f;

    @Override
    public void runOpMode() {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap, telemetry, gamepad1, this);
        CommonLibrary cl = new CommonLibrary();
        cl.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            if (runLinearCode) {
                telemetry.update();
                vuforiaPosition = al.pictoDecipher(telemetry, this);
                al.closeArms(cl, this);
                telemetry.addLine("Endng close arms");
                al.decipherJewelAndKnockOff(telemetry, this, cl);
                al.robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                telemetry.addData("vuforia ", vuforiaPosition);
                telemetry.update();

                if (al.teamColorAndPosition == 1) {

                    //red team corner balance board
                    telemetry.addLine("I am on the red team and in the corner");
                    telemetry.update();
                    al.driveAtAngle(26, 270, telemetry, this);
                    al.turnToAngleWithPID(90, telemetry, this);
                    al.driveAtAngle(6, 90, telemetry, this);
                    al.driveToVuforiaPositionFromTheRight(telemetry, this, vuforiaPosition);
                } else if (al.teamColorAndPosition == 2) {

                    //red ream center balance board
                    telemetry.addLine("I am on the red team and in the center");
                    telemetry.update();
                    cl.wait(200, this);
                    al.driveAtAngle(22, 270, telemetry, this);
                    cl.wait(200, this);
                    al.turnToAngleWithPID(180, telemetry, this);
                    cl.wait(200, this);
                    al.driveToVuforiaPositionFromTheRight(telemetry, this, vuforiaPosition);
                } else if (al.teamColorAndPosition == 3) {

                    //blue team corner balance board
                    telemetry.addLine("I am on the blue team and in the corner");
                    telemetry.update();
                    al.driveAtAngle(24, 90, telemetry, this);
                    al.turnToAngleWithPID(90, telemetry, this);
                    al.driveToVuforiaPositionFromTheLeft(telemetry, this, vuforiaPosition);
                } else if (al.teamColorAndPosition == 4) {

                    //blue team center balance board
                    telemetry.addLine("I am on the blue team and in the center");
                    telemetry.update();
                    al.driveAtAngle(24, 90, telemetry, this);
                    cl.wait(200, this);
                    al.driveToVuforiaPositionFromTheLeft(telemetry, this, vuforiaPosition);
                } else {

                    telemetry.addLine("I don't know where I am who I am what's going on");
                    telemetry.update();
                }

               // al.driveAtAngle(4, 90, telemetry, this);
                al.openArms();
                al.driveAtAngle(14, 90, telemetry, this);//push block in more
                al.driveAtAngle(6, 270, telemetry, this);
            }

            runLinearCode = false;
        }
    }
}