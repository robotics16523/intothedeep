package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;//andrew tate is a goofy goober.

@Autonomous(name="AutonF2toD6-v5", group = "Concept")
public class CenterstageF2toD6 extends LinearOpMode {


    @Override public void runOpMode(){
        RobotHardwareMethods16523 robot = new RobotHardwareMethods16523();
        waitForStart();
        if (opModeIsActive()) {
                robot.init(hardwareMap);
                robot.openGrabber();
                robot.drive(76, .5);
                robot.drive(-65, .5);
                robot.strafeRight(183,.5);
                robot.drive(61,.5);
                robot.pivot(90,.5);
                robot.armPosition(-4275, .5);
                robot.tilterplace();
                // test robot.drive(30.5,.5);
                robot.closeGrabber();
                robot.drive(-10,.5);
                robot.tilterpickup();
                robot.armPosition(-45,0.5);
                robot.strafeLeft(61,.5);
                robot.drive(-30.5,0.5);
            }
        }
    }


