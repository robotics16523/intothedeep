package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="PremierSampleAuto_v60", group = "Concept")
public class PremierSampleAuto extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.closeGrabber();
            robot.tiltUp();
            robot.timeOut(.1);
            robot.strafeRight(5,.6);
            robot.driveForward(21,.55);
            robot.spinLeft(12,.5);
            robot.timeOut(.25);
            robot.closeGrabber();
            robot.lowerArm(.9,2.4);// lower arm and raise arm are swapped
            robot.tiltMiddle();
            robot.timeOut(.2);
            robot.openGrabber();
            robot.lowerArm(.9,.3);
            robot.timeOut(.3);
            //first sample ends here
            robot.tiltUp();
            robot.driveBackward(5,.9);
            robot.raiseArm(.9,.5);
            robot.spinRight(64,.6);
            robot.driveForward(29,.6);
            robot.openGrabber();
            robot.tiltDown();
            robot.timeOut(.6);
            robot.closeGrabber();
            robot.timeOut(.4);
            robot.tiltUp();
            robot.timeOut(.3);
            robot.lowerArm(.9,.1);
            //second sample ends here
            robot.driveBackward(22,.7);
            robot.spinLeft(70,.6);
            robot.driveForward(10,.7);
            robot.lowerArm(.9,2.5);
            robot.tiltMiddle();
            robot.timeOut(.2);
            robot.openGrabber();
            robot.lowerArm(.9,.3);
            robot.tiltUp();
            robot.timeOut(.3);
            robot.raiseArm(.9,.5);
            robot.spinRight(65,.6);
            robot.driveForward(25,.6);
            robot.openGrabber();
            robot.tiltDown();
            robot.timeOut(.6);
            robot.closeGrabber();
            robot.timeOut(.3);
            robot.tiltUp();
            robot.lowerArm(.9,.1);
            robot.driveBackward(25,.6);
            robot.spinLeft(67,.7);
            robot.driveForward(2,.7);
            robot.lowerArm(.9,2.5);
            robot.tiltMiddle();
            robot.timeOut(.2);
            robot.openGrabber();
            robot.lowerArm(.9,.3);
            robot.tiltUp();
            robot.timeOut(.5);
            robot.raiseArm(.9,.5);
            robot.spinRight(50,.7);
            robot.driveForward(15,.7);




        }
    }
}
