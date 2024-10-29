package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoMiddlePushPark2_v2", group = "Concept")
public class AutoMiddlePushPark2 extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.driveForward(robot.FIELD_TILE/5,0.75);
            robot.strafeLeft(robot.FIELD_TILE*2,0.75);
            robot.strafeRight(robot.FIELD_TILE/4,0.75);
            robot.driveForward(robot.FIELD_TILE,0.75);
            robot.strafeRight(robot.FIELD_TILE*5,0.75);
            robot.driveBackward(robot.FIELD_TILE,0.75);
        }
    }
}
// This works!! Copy and paste it to other auto codes.
