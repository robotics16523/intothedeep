package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoF3F6PushPark1", group = "Concept")
public class F3F6PushPark1 extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.driveForward(robot.FIELD_TILE/3,0.75);
            robot.strafeLeft(robot.FIELD_TILE*2,0.75);
            robot.strafeRight(robot.FIELD_TILE*5,0.75);
        }
    }
}
// This works!! Copy and paste it to other auto codes.
//A is simplest - ask maitreyi if you have questions