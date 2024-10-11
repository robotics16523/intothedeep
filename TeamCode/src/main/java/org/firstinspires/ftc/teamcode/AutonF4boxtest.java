package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

public class AutonF4boxtest {
    @Autonomous(name = "Auton A2 2nd Rigging Spot 2 v6", group = "robot")
    public class Auton_A2_2nd_Rigging_Parking_Spot_2 extends LinearOpMode {
        /* Declare OpMode members. */
        RobotMethods robot = new RobotMethods();
        @Override
        public void runOpMode() {
            robot.init(hardwareMap);
            waitForStart();
            robot.driveForward(60.95,1);
            robot.strafeLeft(60.95,1);
            robot.driveBackward(60.95,1);
            robot.strafeRight(60.95,1);
        }
    }
}
