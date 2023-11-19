package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutonF2toF6", group = "Concept")
public class CenterstageF2toF6 extends LinearOpMode {


    @Override public void runOpMode(){
        RobotHardwareMethods16523 robot = new RobotHardwareMethods16523();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.openGrabber();
            robot.drive(61, .5);
            robot.drive(-61, .5);
            robot.armPosition(-757,.5);
            robot.strafeRight(183,.5);
            robot.drive(61,.5);
            robot.rotate(90,.5);
            robot.armPosition(-4275, .5);
            robot.tilterplace();
            robot.drive(30.5,.5);
            robot.closeGrabber();
            robot.drive(-10,.5);
            robot.tilterpickup();
            robot.armPosition(-15,0.5);
            robot.strafeRight(61,.5);
            robot.rotate(-180,.5);
            robot.drive(-30.5,0.5);
        }
    }
}


