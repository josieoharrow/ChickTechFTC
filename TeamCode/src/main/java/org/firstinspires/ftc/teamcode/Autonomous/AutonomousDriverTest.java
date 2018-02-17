package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.pictoDecipher;
//import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousLibrary.initial;


/**
 * Created by Robotics on 8/27/2017.
 */


@Autonomous(name = "Test Autonomous")
//@Disabled
public class AutonomousDriverTest extends LinearOpMode {

    boolean runLinearCode = true;
    boolean setFirstBlockInFarLeftColumn = false;
    String vuforiaPosition = "unknown";

    static float JEWEL_ACTUATOR_UP = 0.2f;
    RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap, telemetry, gamepad1, this);
        CommonLibrary cl = new CommonLibrary();
        cl.init(hardwareMap);
        Boolean ran = false;
        waitForStart();
        while (opModeIsActive()) {

            try {

                if (runLinearCode) {

                    telemetry.update();
                    vuforiaPosition = al.pictoDecipher(telemetry, this);
                    al.closeArms(cl, this);
                    telemetry.addLine("Endng close arms");
                    al.decipherJewelAndKnockOff(telemetry, this, cl);
                    al.robot.jewelActuatorServo.setPosition(JEWEL_ACTUATOR_UP);
                    telemetry.addData("vuforia ", vuforiaPosition);
                    telemetry.update();
                    cl.wait(400, this);
                    if (vuforiaPosition == "left") {
                        setFirstBlockInFarLeftColumn = true;
                    }
                    if (al.teamColorAndPosition == 1) {

                        //red team corner balance board
                        telemetry.addLine("I am on the red team and in the corner");
                        telemetry.update();
                        cl.wait(200, this);
                        al.driveAtAngle(23, 270, telemetry, this);
                        cl.wait(200, this);
                        al.PIDturnRelativeToField(90, telemetry, this);//bump up
                        cl.wait(300, this);
                        //al.driveAtAngle(3, 0, telemetry, this);
                        al.driveToVuforiaPositionFromTheRight(telemetry, this, vuforiaPosition);
                    } else if (al.teamColorAndPosition == 2) {

                        //red ream center balance board
                        telemetry.addLine("I am on the red team and in the center");
                        telemetry.update();
                        //cl.wait(200, this);
                        al.driveAtAngle(22, 270, telemetry, this);
                        //cl.wait(200, this);
                        cl.wait(200, this);
                        al.PIDturnRelativeToField(180, telemetry, this);//bump up when fixed
                        cl.wait(300, this);

                        al.driveAtAngle(1, 180, telemetry, this);
                        al.driveToVuforiaPositionFromTheRight(telemetry, this, vuforiaPosition);
                    } else if (al.teamColorAndPosition == 3) {

                        //blue team corner balance board
                        telemetry.addLine("I am on the blue team and in the corner");
                        telemetry.update();
                        al.driveAtAngle(23.5, 90, telemetry, this);//4
                        cl.wait(200, this);
                        al.PIDturnRelativeToField(90, telemetry, this);
                        cl.wait(300, this);
                        al.driveToVuforiaPositionFromTheLeft(telemetry, this, vuforiaPosition);
                    } else if (al.teamColorAndPosition == 4) {

                        //blue team center balance board
                        telemetry.addLine("I am on the blue team and in the center");
                        telemetry.update();
                        al.driveAtAngle(24, 90, telemetry, this);
                        cl.wait(200, this);
                        al.PIDturnRelativeToField(0, telemetry, this);//bump up when fixed
                        cl.wait(200, this);
                        al.driveAtAngle(2, 0, telemetry, this);
                        cl.wait(300, this);

                        //cl.wait(200, this);
                        al.driveToVuforiaPositionFromTheLeft(telemetry, this, vuforiaPosition);
                    } else {

                        telemetry.addLine("I don't know where I am or who I am. What's going on?!");
                        telemetry.update();
                    }

                    al.driveAtAngle(12, 90, telemetry, this);   //May need to change
                    al.rollerSpinOut(5, telemetry, this);   //May need to change time
                    al.driveAtAngle(7, 270, telemetry, this);   //Too far?

                }

                runLinearCode = false;
            }

            catch (Exception e) {

                telemetry.addLine("Autonomous was interrupted.");
                telemetry.update();
            }
        }
    }
}
