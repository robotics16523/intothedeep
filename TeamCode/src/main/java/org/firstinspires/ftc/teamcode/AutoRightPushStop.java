package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoRightPushStop", group = "Concept")
public class AutoRightPushStop extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.driveForward(robot.FIELD_TILE/5,0.75);
            robot.strafeLeft(robot.FIELD_TILE*3,0.75);
            robot.strafeRight(robot.FIELD_TILE/8,0.75);
            robot.driveForward(robot.FIELD_TILE/8,0.75);
        }
    }
}
// cleared for comp
