package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoF2F6PushPark1", group = "Concept")
public class AutoF2F6PushPark1 extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
           robot.driveForward(robot.FIELD_TILE/3,0.75);
            robot.strafeLeft(robot.FIELD_TILE,0.75);
            robot.strafeRight(robot.FIELD_TILE*5,0.75);
        }
    }
}
// 1 is always the simplest, but just ask Maitreyi if you aren't sure which program is which
//Maps of auto are coming soon
// This works!! Copy and paste it to other auto codes.
