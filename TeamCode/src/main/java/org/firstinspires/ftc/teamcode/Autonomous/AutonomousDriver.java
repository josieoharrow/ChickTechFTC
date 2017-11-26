package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by Robotics on 8/27/2017.
 */
@Autonomous(name = "Main Autonomous")
//@Disabled
public class AutonomousDriver extends LinearOpMode {

    boolean runLinearCode = true;
    int teamColor;
    String vuforiaPosition = "unknown";

    @Override
    public void runOpMode() {

        AutonomousLibrary al = new AutonomousLibrary();
        al.init(hardwareMap, telemetry, gamepad1, this);
        waitForStart();

        while (opModeIsActive()) {

            if (runLinearCode) {

           //     teamColor = al.setTeamColor(telemetry);
                al.decipherJewelAndKnockOff(telemetry, this);
                al.robot.jewelActuatorServo.setPosition(0.3);
                // al.driveAtAngle(18, 135, telemetry, this);// switch to unit circle
               /* if (teamColor == 1) {
                    al.driveAtAngle(25, 120, telemetry, this);
                } else {
                    al.driveAtAngle(25, 30, telemetry, this);
                }*/







             //   al.driveAtAngle(18, 270, telemetry, this);
                //al.decipherJewelAndKnockOff(telemetry, this);
                //al.driveAtAngle(18, 270, telemetry, this);// switch to unit circle

                /*
                //al.decipherJewelAndKnockOff(telemetry, this);
                if (al.robot.isRed == 1){
                    telemetry.addLine("I am on the red team! :D");
                    telemetry.update();
                    //al.driveAtAngle();

                } else if (al.robot.isRed == 0){
                    telemetry.addLine("I am on the blue team! :D");
                    telemetry.update();
                    //al.driveAtAngle();

                } else {
                    telemetry.addLine("I had troubles communicating with my color sensor and I don't know what team I am on");
                    telemetry.update();
                    //wouldn't it be so cool to add LEDs here later in the season where it would flash for errors
                }

                /*telemetry.addLine("First drive");
                telemetry.update();
                al.driveAtAngle(18, 45, telemetry, this);

                telemetry.addLine("Vuforia");
                telemetry.update();
                al.pictoDecipher(telemetry, this);
                telemetry.addLine("2nd drive");
                telemetry.update();
                al.driveAtAngle(12, 70, telemetry, this);*/
            }

            runLinearCode = false;
        }
    }
}