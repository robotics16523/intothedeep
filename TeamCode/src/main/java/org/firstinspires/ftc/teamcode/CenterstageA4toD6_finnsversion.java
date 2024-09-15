package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    @Autonomous(name = "Auton-A4toD6-finnsversion-v2", group = "Concept")
    public class CenterstageA4toD6_finnsversion extends LinearOpMode {
        RobotMethods robot = new RobotMethods();
        @Override
        public void runOpMode() {
            robot.init(hardwareMap);
            //edit driveformula1 and2 the values may not actually match the playing field, i created them just by a simple diagram of the map.
            double driveFormula1 = (robot.SQUARE_LENGTH)+(robot.SQUARE_LENGTH);
            double driveFormula2 = (robot.SQUARE_LENGTH*0.15);
            double armFormula1 = (robot.ARM_MAXIMUM/2);
            double armFormula2 = (robot.ARM_MAXIMUM/10);
            robot.drive(driveFormula1,.5);
            robot.strafe(-driveFormula1,.5);
            robot.pivot(90,.5);
            robot.drive(driveFormula2,.5);
            robot.moveArmToPosition(0.5, (int) armFormula1);
            robot.tilterplace();
            robot.openGrabber();
            robot.moveArmToPosition(0.5,(int)armFormula2);
            robot.tilterpickup();
            robot.drive(-driveFormula1,.5);
        }

    }
