package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoArmTest2", group = "Concept")
public class AutoArmTest2 extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.driveForward(30,1);
           robot.tilter(0.62,1.5);

            //  robot.lowerArm(0.5,1.5);
        }
    }
}
