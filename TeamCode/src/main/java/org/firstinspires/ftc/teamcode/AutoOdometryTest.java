package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Odometry Test v1", group = "Concept")
public class AutoOdometryTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        RobotMethods robot = new RobotMethods();

        if (opModeInInit()){
            robot.init(hardwareMap);
        }
        waitForStart();
        if (opModeIsActive()) {

           robot.odometer.resetPosAndIMU();

           robot.runToOdometerPosition(new Pose2D(DistanceUnit.CM,10,10, AngleUnit.DEGREES,0),1);






        }
    }
}
// this works! copy aND PASTE TO OTHER AUTOS