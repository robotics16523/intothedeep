/* Copyright (c) 2022 FIRST. All rights reserved.
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


//CHARGE CONTROL HUBS
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
// Declare all opmode members here
public class RobotMethods {
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor hangingMotor = null;
    public DcMotor arm = null;
    public Servo tilter = null;
    public CRServo intake = null;
    public Servo wrist = null;
    public final double COUNTS_PER_MOTOR_REV = 537.7;
    public final double DRIVE_WHEEL_DIAMETER_CENTIMETERS = 9.6;
    double strafeTick = (COUNTS_PER_MOTOR_REV / (Math.PI * DRIVE_WHEEL_DIAMETER_CENTIMETERS));
    public final double FIELD_TILE = 60.96; //square centimeters
    public final double DRIVE_GEAR_REDUCTION = 1.001;
    public final double DRIVE_COUNTS_PER_CENTIMETERS = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER_CENTIMETERS * Math.PI);
    public final double DRIVE_WHEEL_DIAMETER_MM = 96.0; // Diameter of the wheel
    public final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (Math.PI * DRIVE_WHEEL_DIAMETER_MM / 10); // Convert from mm to cm
    public final double WHEEL_BASE_WIDTH_CM = 2 * DRIVE_WHEEL_DIAMETER_MM / 10; // Convert from mm to cm
    public final double COUNTS_PER_DEGREE = COUNTS_PER_CM * Math.PI * WHEEL_BASE_WIDTH_CM / 360.0;
    public boolean armIsGoingDown = false;
    public final double TILTER_UP = .62;
    public final double TILTER_DOWN =.25;
   // public final double NEUTRAL = 0;

    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        hangingMotor = hardwareMap.get(DcMotor.class, "hangingMotor");
        arm = hardwareMap.get(DcMotor.class,"arm");
        tilter = hardwareMap.get(Servo.class, "tilter"); //also known as "elbow"
        intake = hardwareMap.get(CRServo.class, "intake");//map this to robot config as of 10/31/24
        wrist = hardwareMap.get(Servo.class,"wrist");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        hangingMotor.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);
    }


//    public boolean isIntakeMoving() {
//        double lastPosition = 0;
//        long lastTime = 0;
//        double currentPosition = intake.getPosition();
//        long currentTime = System.currentTimeMillis();
//
//        // Calculate the change in position and time
//        double positionChange = Math.abs(currentPosition - lastPosition);
//        long timeChange = currentTime - lastTime;

//        // Update last position and time
//        lastPosition = currentPosition;
//        lastTime = currentTime;
//
//        // Define a threshold for movement (this may need to be tuned)
//        return positionChange > 0.01 && timeChange < 200; // Change threshold as necessary
//    }
    public void spinLeft(double distance, double power) {
        int leftfronttarget = leftFrontDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int leftbacktarget = leftBackDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightfronttarget = rightFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightbacktarget = rightBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);

        leftFrontDrive.setTargetPosition(leftfronttarget);
        leftBackDrive.setTargetPosition(leftbacktarget);
        rightBackDrive.setTargetPosition(rightbacktarget);
        rightFrontDrive.setTargetPosition(rightfronttarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(Math.abs(power) * 0.5);
        leftBackDrive.setPower(Math.abs(power) * 0.5);
        rightFrontDrive.setPower(Math.abs(power) * 0.5);
        rightBackDrive.setPower(Math.abs(power) * 0.5);

        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {

        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void spinRight(double distance, double power) {
        int leftfronttarget = leftFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int leftbacktarget = leftBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightfronttarget = rightFrontDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightbacktarget = rightBackDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);

        leftFrontDrive.setTargetPosition(leftfronttarget);
        leftBackDrive.setTargetPosition(leftbacktarget);
        rightBackDrive.setTargetPosition(rightbacktarget);
        rightFrontDrive.setTargetPosition(rightfronttarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(Math.abs(power) * 0.5);
        leftBackDrive.setPower(Math.abs(power) * 0.5);
        rightFrontDrive.setPower(Math.abs(power) * 0.5);
        rightBackDrive.setPower(Math.abs(power) * 0.5);

        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {

        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extendHangingMotor(double power) {
        hangingMotor.setPower(Math.abs(power));
    }

    public void retractHangingMotor(double power) {
        hangingMotor.setPower(-(Math.abs(power)));
    }


    public void reverseArmControls() {
        armIsGoingDown = true;
    }

    public void strafe(double distance, double power) {
        int leftfronttarget = leftFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS); //offset
        int leftbacktarget = leftBackDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightfronttarget = rightFrontDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightbacktarget = rightBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);

        leftFrontDrive.setTargetPosition(leftfronttarget);
        leftBackDrive.setTargetPosition(leftbacktarget);
        rightBackDrive.setTargetPosition(rightbacktarget);
        rightFrontDrive.setTargetPosition(rightfronttarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = 0;
        max = Math.max(max, power);
        if (max > 1.2) {
            power /= max;
        }

        leftFrontDrive.setPower(Math.abs(power) * 0.5);
        leftBackDrive.setPower(Math.abs(power) * 0.5);
        rightFrontDrive.setPower(Math.abs(power) * 0.5);
        rightBackDrive.setPower(Math.abs(power) * 0.5);


        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) { //invert syntax?

        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        /*leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

    }

    public void strafeRight(double distanceCm, double power) {
        double strafeRightDistance = (Math.abs(distanceCm));
        strafe(strafeRightDistance, power);
    }

    public void strafeLeft(double distanceCm, double power) {
        double strafeLeftDistance = -(Math.abs(distanceCm));
        strafe(strafeLeftDistance, power);
    }

    public void drive(double distance, double power) {
        int leftfronttarget = leftFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int leftbacktarget = leftBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightfronttarget = rightFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightbacktarget = rightBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);


        leftFrontDrive.setTargetPosition(leftfronttarget);
        leftBackDrive.setTargetPosition(leftbacktarget);
        rightBackDrive.setTargetPosition(rightbacktarget);
        rightFrontDrive.setTargetPosition(rightfronttarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(Math.abs(power) * 0.5);
        leftBackDrive.setPower(Math.abs(power) * 0.5);
        rightFrontDrive.setPower(Math.abs(power) * 0.5);
        rightBackDrive.setPower(Math.abs(power) * 0.5);

        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {

        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveForward(double distance, double power) {
        double driveDistance = Math.abs(distance);
        drive(driveDistance, power);

    }

    public void driveBackward(double distance, double power) {
        double driveDistance = -(Math.abs(distance)); //seems redundant but does make sense, fixes user error
        drive(driveDistance, power);
    }

}
