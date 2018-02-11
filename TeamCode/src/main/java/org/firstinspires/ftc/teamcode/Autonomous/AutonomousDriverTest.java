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


@Autonomous(name = "Autonomous with MR Sensor")
//@Disabled
public class AutonomousDriverTest extends LinearOpMode {

    boolean runLinearCode = true;
    String vuforiaPosition = "unknown";
    RobotHardware robot;
    int columnPlacement;
    @Override
    public void runOpMode() throws InterruptedException {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap, telemetry, gamepad1, this);
        CommonLibrary cl = new CommonLibrary();
        cl.init(hardwareMap);
        Boolean ran = false;
        telemetry.addLine("Hey now, Hey now.\n(You can start Autonomous now)");
        telemetry.update();
        waitForStart();
        telemetry.update();
        //al.driveByBlockColumns(this, true, 3);

        vuforiaPosition = al.pictoDecipher(telemetry, this);
        al.halfCloseArms(cl, this);
        al.lowerLift();
        al.closeArms(cl, this);
        telemetry.addLine("Endng close arms");
        al.decipherJewelAndKnockOff(telemetry, this, cl);
        al.robot.jewelActuatorServo.setPosition(0.2f);
        telemetry.addData("vuforia ", vuforiaPosition);
        telemetry.update();
        cl.wait(400, this);

        if (al.teamColorAndPosition == 1) {

            //red team corner balance board

            if (vuforiaPosition == "right") {
                columnPlacement = 1;
            } else if (vuforiaPosition == "left") {
                columnPlacement = 3;
            } else {
                columnPlacement = 2;
            }

            telemetry.addLine("I am on the red team and in the corner");
            telemetry.update();
            cl.wait(200, this);
            al.driveAtAngle(23, 270, telemetry, this);
            cl.wait(200, this);
            al.PIDturnRelativeToField(90, telemetry, this);//bump up
            cl.wait(300, this);
            //al.driveAtAngle(3, 0, telemetry, this);
            al.driveAtAngle(5, 0, telemetry, this);
            cl.wait(200, this);
            al.PIDturnRelativeToField(90, telemetry, this);//bump up

            al.moveLift(-1.3f);
            cl.wait(500, this);

            al.driveByBlockColumns(this, false, columnPlacement);
            al.PIDturnRelativeToField(90, telemetry, this);//bump up

        } else if (al.teamColorAndPosition == 2) {

            //red ream center balance board

            if (vuforiaPosition == "right") {
                columnPlacement = 1;
            } else if (vuforiaPosition == "left") {
                columnPlacement = 3;
            } else {
                columnPlacement = 2;
            }

            telemetry.addLine("I am on the red team and in the center");
            telemetry.update();
            //cl.wait(200, this);
            al.driveAtAngle(22, 270, telemetry, this);
            //cl.wait(200, this);
            cl.wait(200, this);
            al.PIDturnRelativeToField(180, telemetry, this);//bump up when fixed
            cl.wait(300, this);

            al.driveAtAngle(10, 0, telemetry, this);
            cl.wait(200, this);
            al.PIDturnRelativeToField(180, telemetry, this);//bump up

            al.moveLift(-1.3f);
            cl.wait(500, this);

            al.driveByBlockColumns(this, false, columnPlacement);
            al.PIDturnRelativeToField(180, telemetry, this);//bump up

        } else if (al.teamColorAndPosition == 3) {

            //blue team corner balance board

            if (vuforiaPosition == "right") {
                columnPlacement = 3;
            } else if (vuforiaPosition == "left") {
                columnPlacement = 1;
            } else {
                columnPlacement = 2;
            }

            telemetry.addLine("I am on the blue team and in the corner");
            telemetry.update();
            al.driveAtAngle(23.5, 90, telemetry, this);//4
            cl.wait(200, this);
            al.PIDturnRelativeToField(90, telemetry, this);
            cl.wait(300, this);
            al.driveAtAngle(3, 180, telemetry, this);
            cl.wait(200, this);
            al.PIDturnRelativeToField(90, telemetry, this);//bump up

            al.moveLift(-1.3f);
            cl.wait(500, this);

            al.driveByBlockColumns(this, true, columnPlacement);
            al.PIDturnRelativeToField(90, telemetry, this);//bump up

        } else if (al.teamColorAndPosition == 4) {

            //blue team center balance board

            if (vuforiaPosition == "right") {
                columnPlacement = 3;
            } else if (vuforiaPosition == "left") {
                columnPlacement = 1;
            } else {
                columnPlacement = 2;
            }
            telemetry.addLine("I am on the blue team and in the center");
            telemetry.update();
            al.driveAtAngle(24, 90, telemetry, this);
            cl.wait(200, this);
            al.PIDturnRelativeToField(0, telemetry, this);//bump up when fixed
            cl.wait(200, this);
            //al.driveAtAngle(2, 0, telemetry, this);
            cl.wait(300, this);
            al.driveAtAngle(5, 180, telemetry, this);
            al.PIDturnRelativeToField(0, telemetry, this);//bump up

            cl.wait(200, this);
            //cl.wait(200, this);
            al.moveLift(-1.3f);
            cl.wait(500, this);
            al.driveByBlockColumns(this, true, columnPlacement);
            al.PIDturnRelativeToField(0, telemetry, this);//bump up

        } else {

            telemetry.addLine("I don't know where I am who I am what's going on");
            telemetry.update();
        }

        al.openArms(cl, this);
        al.driveAtAngle(12, 90, telemetry, this);//push block in more
        al.driveAtAngle(7, 270, telemetry, this);
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

}
