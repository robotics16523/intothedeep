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
            robot.grabber(robot.GRABBER_CLOSED);
            robot.timeOut(0.5);
            robot.tilter(robot.TILTER_UP,2);
            robot.timeOut(1);
            robot.driveForward(robot.FIELD_TILE,0.75);
            robot.strafeLeft(robot.FIELD_TILE/2,0.75);
            robot.raiseArm(0.75,1);
            robot.timeOut(1);
          robot.driveForward(robot.FIELD_TILE/4.5,0.75);
            robot.timeOut(1);
            robot.lowerArm(0.75,0.45);
           robot.grabber(robot.GRABBER_OPEN);
           robot.timeOut(1);
           robot.driveBackward(robot.FIELD_TILE/2,0.75);
           robot.lowerArm(0.75,0.55);
           robot.strafeLeft(robot.FIELD_TILE*2,0.75);
           robot.driveForward(robot.FIELD_TILE*2,0.75);
           robot.strafeRight(robot.FIELD_TILE,0.75 );
            //  robot.lowerArm(0.5,1.5);
        }
    }
}
