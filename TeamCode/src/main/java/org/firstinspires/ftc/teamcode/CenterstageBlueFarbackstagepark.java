package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auton-BlueFarPark-v", group = "Concept")
public class CenterstageBlueFarbackstagepark extends LinearOpMode {

    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            try {
                robot.init(hardwareMap);
                robot.drive(7, .5);
                Thread.sleep(400);
                //robot.strafe(Math.round(240), .5)
                robot.strafeLeft(240,.5);
                Thread.sleep(6000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

    }

}
