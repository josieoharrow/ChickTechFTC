package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Robotics on 8/27/2017.
 */
public class AutonomousLibrary {

    RobotHardware robot;
    HardwareMap hardwareMap;
    static VuforiaLocalizer vuforia;
    static String pictoKey = "unknown";
    static String vuMarkSeen = "no";

    public static void initial (HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac+j+R7/////AAAAGXEMop5pnkoqqEXMkOojnpQriKcyqCStGTQ0SVWtZDKiyucL+bWQPvA2YRrhGk/diKOkLGVRsP2l0UHYI37HSgl59Y81KNpEjxUEj34kk/Tm+ck3RrCgDuNtY4lsmePAuTAta6jakcmmESS4Gd2e0FAI97wuo6uJ4CAOXeAFs+AcqNQ162w10gJqOaTlYJVU1z8+UWQca/fwc/pcQ4sqwXzsL3NFpMgE3cijkAGxIZ6xAxkK5YI+3QJxzljDhszlG8dVOx8JJ4TflpzMNYpya36bPiKUlT++LQb6Xmn+HJpOChXg3vEtp2TV9hkFCe1CNjoYFCpsMTORho4tUGNPeUK0+JQBnHozcnbJdVnV+e/L";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    }


    public void declareRobot(RobotHardware robotSent) {

        robot = robotSent;
    }

    public void init() {
        robot.init(hardwareMap);
    }

    public void motorsOn(){
        robot.frontMotor.setPower(1);
    }

    public static void pictoDecipher(Telemetry telemetry, HardwareMap hardwareMap){

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while(vuMarkSeen == "no") {

            if (vuMark == RelicRecoveryVuMark.UNKNOWN){
                pictoKey = "unknown";
                vuMarkSeen = "no";
                telemetry.addData("VuMark", "not visible");
                telemetry.update();

            }
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                pictoKey = "left";
                vuMarkSeen = "yes";
                telemetry.addData("VuMark", "is visible:", vuMark, pictoKey);
                telemetry.update();

            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                pictoKey = "center";
                vuMarkSeen = "yes";
                telemetry.addData("VuMark", "is visible:", vuMark, pictoKey);
                telemetry.update();

            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                pictoKey = "right";
                vuMarkSeen = "yes";
                telemetry.addData("VuMark", "is visible:", vuMark, pictoKey);
                telemetry.update();

            }
            if (vuMarkSeen == "yes"){
                break;

            }

        }

    }
}
