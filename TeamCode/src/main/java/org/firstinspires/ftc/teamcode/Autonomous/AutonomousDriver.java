package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.CommonLibrary;


/**
 * Created by Robotics on 8/27/2017.
 */
@Autonomous(name = "Autonomous NO MR Sensor")
//@Disabled

public class AutonomousDriver extends LinearOpMode {

    boolean runLinearCode = true;
    boolean setFirstBlockInFarLeftColumn = false;
    String vuforiaPosition = "unknown";

    static float JEWEL_ACTUATOR_UP = 0.2f;

    @Override
    public void runOpMode() {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap, telemetry, gamepad1, this);
        CommonLibrary cl = new CommonLibrary();
        cl.init(hardwareMap);
        telemetry.addLine("ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            try {

                if (runLinearCode) {

                    telemetry.update();
                    vuforiaPosition = al.pictoDecipher(telemetry, this);
                    al.halfCloseArms(cl, this);
                    al.lowerLift(this);
                    al.closeArms(cl, this);
                    al.moveLift(1, this);
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

                        telemetry.addLine("I don't know where I am who I am what's going on");
                        telemetry.update();
                    }

                    al.moveLift(-1.5f, this);

                    al.openArms(cl, this);
                    al.driveAtAngle(12, 90, telemetry, this);//push block in more
                    al.driveAtAngle(7, 270, telemetry, this);
                    /*al.turnToAngleWithPID(90, telemetry, this);//Turn so arms won't hit block
                    al.driveAtAngle(15, 0, telemetry, this);
                    al.driveAtAngle(10, 180, telemetry, this);
                   // al.driveAtAngle(10, 90, telemetry, this);
                   /* final AutonomousLibrary newAl;
                    newAl = al;

                    Thread t1 = new Thread(new Runnable() {
                        public void run() {

                            newAl.lowerLift();
                        }
                    });

                    t1.start();*/
                }

                runLinearCode = false;
            } catch (Exception e) {

                telemetry.addLine("Autonomous was interrupted.");
                telemetry.update();
            }
        }
    }
}