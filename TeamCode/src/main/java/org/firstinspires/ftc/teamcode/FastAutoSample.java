package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="FastAutoSample_v6", group = "Concept")
public class FastAutoSample extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
        //    robot.tilt(robot.TILTER_UP,.1);
            robot.tiltUp();
            robot.timeOut(.1);
            robot.strafeRight(45,0.9);
            robot.spinLeft(24,0.9);
            robot.driveForward(25,0.9);
            robot.raiseArm(0.8,2);
            // timeout was here
            //robot.tilt(robot.TILTER_MIDDLE,0.75);
            robot.tiltMiddle();
            robot.timeOut(.2);
            //robot.grab(robot.GRABBER_AUTO_OPEN);
            robot.openAutoGrabber();
            robot.timeOut(.1);
            //robot.tilt(robot.TILTER_UP,0.75);
            robot.tiltUp();
            robot.driveBackward(10,0.9);
            robot.lowerArm(0.8,2);
            robot.spinRight(68,0.9);
            robot.driveForward(13,0.9);
            //robot.tilt(robot.TILTER_DOWN,0.75);
            robot.tiltDown();
            //robot.grab(robot.GRABBER_CLOSED);
            robot.closeGrabber();
            robot.timeOut(.1);
            //robot.tilt(robot.TILTER_UP,0.75);
            robot.tiltUp();
            robot.spinLeft(65,0.9);
            robot.driveForward(25,0.9);
            robot.raiseArm(0.8,2);
            //robot.tilt(robot.TILTER_MIDDLE,0.75);
            robot.tiltMiddle();
            robot.timeOut(.25);
            //robot.grab(robot.GRABBER_AUTO_OPEN);
            robot.openAutoGrabber();
            robot.timeOut(.1);
            //robot.tilt(robot.TILTER_UP,0.75);
            robot.tiltUp();
            robot.driveBackward(12,0.9);
            robot.lowerArm(0.8,2);
//          robot.spinLeft(60,0.75);
//          robot.lowerArm(0.5,3.6);
//          robot.spinRight(80,0.75);
//          robot.driveForward(robot.FIELD_TILE/2,0.75);
//          robot.tilter(0.62,1);

            //  robot.lowerArm(0.5,1.5);
        }
    }
}
// this works!
