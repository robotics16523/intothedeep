package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoMiddlePushStop", group = "Concept")
public class AutoMiddlePushStop extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.driveForward(robot.FIELD_TILE/5,0.75);
            robot.strafeLeft(robot.FIELD_TILE*2,0.75);
            robot.strafeRight(robot.FIELD_TILE/8,0.75);
            robot.driveForward(robot.FIELD_TILE,0.75);
        }
    }
}
// cleared for comp