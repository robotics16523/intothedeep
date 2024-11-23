package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoArmTest_v6", group = "Concept")
public class AutoArmTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);

            robot.tilter(robot.TILTER_UP,1);
            robot.timeOut(1);
       robot.strafeRight(58,0.65);
       robot.spinLeft(27,0.75);
            robot.driveForward(45,0.5);
          robot.raiseArm(0.75,2.2);
          // timeout was here
          robot.tilter(robot.TILTER_MIDDLE,0.75);
            robot.timeOut(.25);
            robot.grabber(robot.GRABBER_OPEN);
          robot.timeOut(.5);
          robot.tilter(robot.TILTER_UP,0.75);
          robot.driveBackward(16,0.75);
          robot.lowerArm(0.75,2.2);
          robot.spinRight(72,0.5);
          robot.driveForward(15,0.5);
          robot.tilter(robot.TILTER_DOWN,0.75);
          robot.grabber(robot.GRABBER_CLOSED);
          robot.timeOut(.5);
          robot.tilter(robot.TILTER_UP,0.75);
          robot.spinLeft(78,0.75);
          robot.driveForward(28,0.75);
          robot.raiseArm(0.75,2.2);
          robot.tilter(robot.TILTER_MIDDLE,0.75);
          robot.timeOut(.25);
          robot.grabber(robot.GRABBER_OPEN);
          robot.timeOut(.5);
          robot.tilter(robot.TILTER_UP,0.75);
          robot.driveBackward(20,0.5);
          robot.lowerArm(0.75,2.2);
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