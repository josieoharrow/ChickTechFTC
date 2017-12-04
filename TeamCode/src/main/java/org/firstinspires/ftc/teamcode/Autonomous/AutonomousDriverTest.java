package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    String vuforiaPosition = "unknown";
    RobotHardware robot;

    @Override
    public void runOpMode () throws InterruptedException {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap, telemetry, gamepad1, this);
        robot = new RobotHardware();
        Boolean ran = false;
        robot.init(hardwareMap);
        waitForStart();


        while (opModeIsActive()) {

            if (!ran) {
                al.blockFollow(this);
                ran = true;
            }

        }

                /*telemetry.update();
                al.closeArms();
               // telemetry.addLine("Endng close arms");
             //   al.decipherJewelAndKnockOff(telemetry, this);
           //     al.robot.jewelActuatorServo.setPosition(0.3);
                vuforiaPosition = al.pictoDecipher(telemetry, this);
                telemetry.addData("vuforia ", vuforiaPosition);
                telemetry.update();

                if (al.teamColorAndPosition == 1) {

                    //red team corner balance board
                    telemetry.addLine("I am on the red team and in the corner");
                    telemetry.update();
                    al.turnToAngleWithPID(-90, telemetry, this);
                    al.driveAtAngle(24, 180, telemetry, this);
                    al.driveAtAngle(6, 90, telemetry, this);
                    al.driveToVuforiaPositionFromTheRight(telemetry, this, vuforiaPosition);
                } else if (al.teamColorAndPosition == 2) {

                    //red ream center balance board
                    telemetry.addLine("I am on the red team and in the center");
                    telemetry.update();
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    al.driveAtAngle(22, 270, telemetry, this);
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    al.turnToAngleWithPID(180, telemetry, this);
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    // al.driveAtAngle(14, 90, telemetry, this);
                    // Thread.sleep(200);
                    al.driveToVuforiaPositionFromTheRight(telemetry, this, vuforiaPosition);
                } else if (al.teamColorAndPosition == 3) {

                    //blue team corner balance board
                    telemetry.addLine("I am on the blue team and in the corner");
                    telemetry.update();
                    al.turnToAngleWithPID(90, telemetry, this);
                    al.driveAtAngle(24, 0, telemetry, this);
                    al.driveAtAngle(6, 90, telemetry, this);
                    al.driveToVuforiaPositionFromTheLeft(telemetry, this, vuforiaPosition);
                } else if (al.teamColorAndPosition == 4) {

                    //blue team center balance board
                    telemetry.addLine("I am on the blue team and in the center");
                    telemetry.update();
                    al.driveAtAngle(24, 90, telemetry, this);
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    al.driveToVuforiaPositionFromTheLeft(telemetry, this, vuforiaPosition);
                } else {

                    telemetry.addLine("I don't know where I am who I am what's going on");
                    telemetry.update();
                }

                // al.driveAtAngle(4, 90, telemetry, this);
                al.openArms();
                al.driveAtAngle(14, 90, telemetry, this);//push block in more
                al.driveAtAngle(6, 270, telemetry, this);
            }*/

            runLinearCode = false;
        }
    }
