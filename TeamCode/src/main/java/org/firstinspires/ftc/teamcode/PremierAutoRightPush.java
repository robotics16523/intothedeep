package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="PremierAutoRightPlace_v1", group = "Concept")
public class PremierAutoRightPush extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.tiltUp();
            robot.driveForward(3,0.7);
            robot.strafeRight(55,0.7);
            robot.timeOut(.5);
            robot.driveForward(110,0.7);
            robot.timeOut(.5);
            robot.strafeRight(30,0.7);
            robot.timeOut(.5);
            robot.driveBackward(100,0.7);
            robot.timeOut(.5);
            robot.driveForward(100,0.7);
            robot.timeOut(.5);
            robot.strafeRight(30,0.7);
            robot.timeOut(.5);
            robot.driveBackward(100,0.7);
            robot.timeOut(.5);
            robot.driveForward(100,0.7);
            robot.timeOut(.5);
            robot.strafeRight(15,0.7);
            robot.timeOut(.5);
            robot.driveBackward(100,0.7);
            robot.timeOut(.5);
            robot.driveForward(20,0.7);
            robot.timeOut(.5);
            robot.driveBackward(20,0.7);
            robot.tiltUp();
        }
    }
}
