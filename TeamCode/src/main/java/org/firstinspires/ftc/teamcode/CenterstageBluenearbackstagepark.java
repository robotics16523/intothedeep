package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auton-BlueNearPark-v11", group = "Concept")
public class CenterstageBluenearbackstagepark extends LinearOpMode {
    RobotHardwareMethods16523 robot = new RobotHardwareMethods16523();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.drive(10, .5);
        robot.strafeLeft(120, .5);
    }

}
