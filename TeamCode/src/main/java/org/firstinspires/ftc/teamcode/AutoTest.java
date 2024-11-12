package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoTEST", group = "Concept")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
        robot.driveForward(robot.FIELD_TILE,0.75);
         //   robot.raiseArm(0.5,3000);
      //  robot.lowerArm(0.5,3000);

        }
    }
}

// This DOES NOT WORK
