package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoASample_v13", group = "Concept")
public class AutoASample extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.tilter(robot.TILTER_UP,0.5);
            robot.timeOut(0.5);
       robot.strafeRight(58,0.65);
       robot.spinLeft(26,0.75);
            robot.driveForward(43,0.5);
          robot.raiseArm(0.75,2.2);
          // timeout was here
          robot.tilter(robot.TILTER_MIDDLE,0.75);
            robot.timeOut(.25);
            robot.grabber(robot.GRABBER_AUTO_OPEN);
          robot.timeOut(.5);
          robot.tilter(robot.TILTER_UP,0.75);
          robot.driveBackward(16,0.75);
          robot.lowerArm(0.75,2.2);
          robot.spinRight(73,0.5);
          // make less later
          robot.driveForward(13.5,0.5); //15
          robot.tilter(robot.TILTER_DOWN,0.75);
          robot.timeOut(0.75);
          robot.grabber(robot.GRABBER_CLOSED);
          robot.timeOut(.5);
          robot.tilter(robot.TILTER_UP,0.75);
          robot.spinLeft(77,0.75);
          robot.driveForward(28.5,0.75);
          robot.raiseArm(0.75,2.2);
          robot.tilter(robot.TILTER_MIDDLE,0.75);
          robot.timeOut(.25);
          robot.grabber(robot.GRABBER_AUTO_OPEN);
          robot.timeOut(.5);
          robot.tilter(robot.TILTER_UP,0.75);
          robot.driveBackward(20,0.5);
          robot.lowerArm(0.75,2.2);
          //robot.tilter(robot.TILTER_DOWN,1);
//          robot.spinLeft(60,0.75);
//          robot.lowerArm(0.5,3.6);
//          robot.spinRight(80,0.75);
//          robot.driveForward(robot.FIELD_TILE/2,0.75);
//          robot.tilter(0.62,1);

          //  robot.lowerArm(0.5,1.5);
        }
    }
}
// this works! copy aND PASTE TO OTHER AUTOS