package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoF4ToPark", group = "Concept")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.driveForward(robot.FIELD_TILE/2,0.75);
            robot.strafeRight(robot.FIELD_TILE*2,0.75);
        }
    }
}

// This works!! Copy and paste it to other auto codes.
