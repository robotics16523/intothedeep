package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auton-BlueNearPark-v2", group = "Concept")
public class CenterstageBluenearbackstagepark extends LinearOpMode {


    @Override public void runOpMode(){
        RobotHardwareMethods16523 robot = new RobotHardwareMethods16523();
        waitForStart();
        if (opModeIsActive()) {
            try {
                robot.init(hardwareMap);
                robot.drive(20, .5);
                Thread.sleep(400);
                robot.strafe(-120, .5);
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

    }

}
