package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="PremierSampleAuto_v22", group = "Concept")
public class PremierSampleAuto extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.closeGrabber();
            robot.lowerArm(.9,2.4);// this works don't change
            robot.tiltMiddle();
            robot.timeOut(.2);
            robot.openGrabber();
            robot.lowerArm(.9,.1);
            robot.timeOut(.5);
            robot.tiltUp();
            robot.driveBackward(10,.55);

            //robot.closeGrabber();
            //robot.tiltUp();
            //robot.timeOut(.1);
            //robot.strafeRight(5,.55);
            //robot.driveForward(24,.55);
            //robot.spinLeft(16,.5);
            //robot.timeOut(.35);
            //robot.closeGrabber();
        }
    }
}
