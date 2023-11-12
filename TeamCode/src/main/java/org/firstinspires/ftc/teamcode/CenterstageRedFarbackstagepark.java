package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auton-RedFarPark-v1", group = "Concept")
public class CenterstageRedFarbackstagepark extends LinearOpMode {


    @Override public void runOpMode(){
        RobotHardwareMethods16523 robot = new RobotHardwareMethods16523();
        waitForStart();
        if (opModeIsActive()){
            robot.init(hardwareMap);
            //robot.drive(20,.5);
            robot.forwardbackwards_tick = 10;
            robot.strafe_tick = -247;
             }


    }

}
