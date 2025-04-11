package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoASampleThatHasParking v2", group = "Concept")
public class AutoASampleThatHasParking extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
           // robot.tilt(robot.TILTER_UP,0.3);
            robot.tiltUp();
            robot.timeOut(0.3);
            robot.strafeRight(58,0.75);
            robot.spinLeft(26,0.75);
            robot.driveForward(45,0.65);
            robot.raiseArm(0.75,2.2);
          // timeout was here
           // robot.tilt(robot.TILTER_MIDDLE,0.5);
            robot.tiltMiddle();
            robot.timeOut(.25);
//            robot.grab(robot.GRABBER_AUTO_OPEN);
            robot.openAutoGrabber();
            robot.timeOut(.5);
        //    robot.tilt(robot.TILTER_UP,0.75);
            robot.tiltUp();
            robot.driveBackward(16,0.75);
            robot.lowerArm(0.75,2.2);
            robot.spinRight(70,0.5);
          // make less later
            robot.driveForward(15,0.75);
          //  robot.tilt(robot.TILTER_DOWN,0.75);
            robot.tiltDown();
            robot.timeOut(0.2);
          //  robot.grab(robot.GRABBER_CLOSED);
            robot.closeGrabber();
            robot.timeOut(.5);
           // robot.tilt(robot.TILTER_UP,0.75);
            robot.tiltUp();
            robot.spinLeft(77,0.75);
            robot.driveForward(30,0.75);
            robot.raiseArm(0.75,2.2);
            //robot.tilt(robot.TILTER_MIDDLE,0.75);
            robot.tiltMiddle();
            robot.timeOut(.25);
          //  robot.grab(robot.GRABBER_AUTO_OPEN);
            robot.openAutoGrabber();
            robot.timeOut(.5);
           // robot.tilt(robot.TILTER_UP,0.75);
            robot.tiltUp();
            robot.driveBackward(20,0.5);
            robot.lowerArm(0.75,2.2);
          //robot.tilter(robot.TILTER_DOWN,1);
//          robot.spinLeft(60,0.75);
//          robot.lowerArm(0.5,3.6);
//          robot.spinRight(80,0.75);
//          robot.driveForward(robot.FIELD_TILE/2,0.75);
//          robot.tilter(0.62,1);

          //  robot.lowerArm(0.5,1.5);
            robot.spinRight(115,0.75);
            robot.driveForward(70,0.75);
            robot.spinRight(14.18,0.75);
            robot.driveForward(35,0.75);
           // robot.grab(robot.GRABBER_CLOSED);
            robot.closeGrabber();
           // robot.tilt(robot.TILTER_DOWN,1);
            robot.tiltDown();
        }
    }
}
// this works! copy aND PASTE TO OTHER AUTOS