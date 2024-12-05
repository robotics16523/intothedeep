/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
// KEEP REV OPEN WHEN PUSHING
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp_v40", group="Linear OpMode")
public class TeleOp extends LinearOpMode {
    RobotMethods robot = new RobotMethods();
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
       // robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
                runtime.reset();
                // run until the end of the match (driver presses STOP)
                while (opModeIsActive()) {
                    double max;
                    // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                    double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives POSITIVE value
                    double lateral =  gamepad1.left_stick_x;
                    double yaw     =  gamepad1.right_stick_x;
                    // Combine the joystick requests for each axis-motion to determine each wheel's power.
                    // Set up a variable for each drive wheel to save the power level for telemetry.
                    double leftFrontPower  = axial + lateral + yaw;
                    double rightFrontPower = axial - lateral - yaw;
                    double leftBackPower   = axial - lateral + yaw;
                    double rightBackPower  = axial + lateral - yaw;

                    // Normalize the values so no wheel power exceeds 100%
                    // This ensures that the robot maintains the desired motion.
                    max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                    max = Math.max(max, Math.abs(leftBackPower));
                    max = Math.max(max, Math.abs(rightBackPower));

                    if (max > 1.0) {
                        leftFrontPower  /= max;
                        rightFrontPower /= max;
                        leftBackPower   /= max;
                        rightBackPower  /= max;
                    }
                    robot.leftFrontDrive.setPower(leftFrontPower*0.65);
                    robot.leftBackDrive.setPower(leftBackPower*0.65);
                    robot.rightFrontDrive.setPower(rightFrontPower*0.65);
                    robot.rightBackDrive.setPower(rightBackPower*0.65);

                    if(gamepad2.left_bumper){
//                        robot.grabber.setDirection(Servo.Direction.REVERSE);
                        robot.grabber.setPosition(robot.GRABBER_OPEN);
                    }
                    else if(gamepad2.right_bumper){
//                        robot.grabber.setDirection(Servo.Direction.FORWARD);
                        robot.grabber.setPosition(robot.GRABBER_CLOSED);
                    }
//                    if(gamepad2.left_trigger > 0){
//                        robot.grabber.setPosition(gamepad2.left_trigger);
//                    }

                    if (gamepad2.dpad_up) {
                        robot.grabber.setPosition(robot.GRABBER_CLOSED);
                        robot.lowerArm(.25,.81);
                        robot.grabber.setPosition(robot.GRABBER_OPEN);
                    }

                    if(gamepad2.y){
                        robot.extendHangingMotor(.75);
                    }
                    if(gamepad2.a){
                        robot.retractHangingMotor(.75);
                    }
                    if(gamepad2.x){
                        robot.tilter.setPosition(robot.TILTER_DOWN);
                    }
                    if(gamepad2.b){
                        robot.tilter.setPosition(robot.TILTER_UP);
                    }
                    if (gamepad2.dpad_left){
                        robot.tilter.setPosition(robot.TILTER_MIDDLE);
                    }
                    if (gamepad2.back){
                        robot.tilter.setPosition(robot.TILTER_MIDDLE);
                    }
                    if(gamepad1.dpad_left){
                        robot.spinLeft(56,.75);
                    }
                    if(gamepad1.dpad_right) {
                        robot.spinRight(56, .75);
                    }
                    if(gamepad1.dpad_up) {
                        robot.spinLeft(112, .75);
                    }
                    double armPower = -gamepad2.left_stick_y;
                    robot.arm.setPower(armPower);
                    if (armPower > 0) {
                        robot.arm.setPower(Math.abs(armPower));
                    }
                   else if (armPower < 0) {
                        robot.arm.setPower(-Math.abs(armPower));
                    }
                   else {
                       robot.arm.setPower(0);
                    }

//                    boolean wristForwardController = gamepad2.right_bumper;
//                    if(wristForwardController) {
//                        robot.wrist.setDirection(Servo.Direction.FORWARD);
//                        robot.wrist.setPosition(.5);
//                    }
//                    boolean wristBackwardController = gamepad2.left_bumper;
//                    if(wristBackwardController) {
//                      robot.wrist.setDirection(Servo.Direction.REVERSE);
//                      robot.wrist.setPosition(0);
//                    }


            // Send calculated power to wheels
            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightBackDrive.setPower(rightBackPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower,    rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm power: ", "%4.2f", armPower);
            telemetry.addData("Arm position:",robot.arm.getCurrentPosition());
            telemetry.addData("Tilter position:",robot.tilter.getPosition());
           telemetry.addData("Grabber position",robot.grabber.getPosition());
            telemetry.update();

        }
    }
}
