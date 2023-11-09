package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auton-v4", group = "Concept")
public class CenterstageAuton16523 extends LinearOpMode {


    @Override public void runOpMode(){
        RobotHardwareMethods16523 robot = new RobotHardwareMethods16523();
        waitForStart();
        if (opModeIsActive()){
            robot.init(hardwareMap);
            //robot.drive(20,.5);
            if(gamepad1.a){
                robot.toggleGrabber();
            }
        }

    }

}