package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auton-BlueFarPark-v", group = "Concept")
public class CenterstageBlueFarbackstagepark extends LinearOpMode {

    public void runOpMode(){
        RobotHardwareMethods16523 robot = new RobotHardwareMethods16523();
        waitForStart();
        if (opModeIsActive()) {
            try {
                robot.init(hardwareMap);
                robot.drive(5, .5);
                Thread.sleep(400);
                //robot.strafe(Math.round(240), .5)
                robot.strafeDirectional("left",240,0.5);
                Thread.sleep(6000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

    }

}
