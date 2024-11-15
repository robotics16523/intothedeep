package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoArmTest", group = "Concept")
public class AutoArmTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();
        waitForStart();
        if (opModeIsActive()) {
            robot.init(hardwareMap);
            robot.strafeLeft(robot.FIELD_TILE*3,0.75);
            robot.driveForward(robot.FIELD_TILE/4,0.75);
            robot.spinLeft(60,0.75);
            robot.driveForward(robot.FIELD_TILE/5,0.75);
          robot.raiseArm(0.5,3.6);
          robot.driveForward(robot.FIELD_TILE/6,0.75);
          //robot.intake("forward",0.9,3 );
            robot.grabber(robot.GRABBER_OPEN);
          robot.timeOut(1);
          robot.driveBackward(robot.FIELD_TILE,0.75);
          robot.spinLeft(60,0.75);
          robot.lowerArm(0.5,3.6);
          robot.spinRight(80,0.75);
          robot.driveForward(robot.FIELD_TILE/2,0.75);
          robot.tilter(0.62,1);

          //  robot.lowerArm(0.5,1.5);
        }
    }
}
// this works! copy aND PASTE TO OTHER AUTOS