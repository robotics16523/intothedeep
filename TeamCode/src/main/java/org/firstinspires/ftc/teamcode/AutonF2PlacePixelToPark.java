package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonF2PlacePixelToPark {
        @Autonomous(name = "AutonF2PlacePixeltoPark1", group = "robot")
        public class AutonF5ToPark extends LinearOpMode {
            RobotMethods robot = new RobotMethods();
            @Override
            public void runOpMode() {
                robot.init(hardwareMap);
                waitForStart();
                robot.strafeRight(robot.SQUARE,0.75);//easiest auton ever ?
            }
        }
    }
