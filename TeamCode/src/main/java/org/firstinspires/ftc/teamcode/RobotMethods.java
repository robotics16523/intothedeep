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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotMethods {
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor hangingMotor = null;
    public DcMotor armMotor = null;
    public Servo tilterServo = null;
    public Servo grabberServo = null;
    public GoBildaPinpointDriver odometer = null;

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
    public final double TILTER_UP = .30;
    public final double TILTER_DOWN = .63;//.445 was original og .635, .6375
    public final double TILTER_MIDDLE = .38; //0.42 og
    public final double GRABBER_OPEN = .5; // previously 0.5
    public final double GRABBER_AUTO_OPEN = 0.45;
    public final double GRABBER_CLOSED = 0.64;


    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        hangingMotor = hardwareMap.get(DcMotor.class, "hangingMotor");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        tilterServo = hardwareMap.get(Servo.class, "tilter");
        grabberServo = hardwareMap.get(Servo.class, "grabber");

        odometer = hardwareMap.get(GoBildaPinpointDriver.class, "odometer");
        odometer.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odometer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        hangingMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive ( double distance, double power){
        int leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int leftBackTarget = leftBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightBackTarget = rightBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);


        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftBackDrive.setTargetPosition(leftBackTarget);
        rightBackDrive.setTargetPosition(rightBackTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(Math.abs(power) * 0.75);
        leftBackDrive.setPower(Math.abs(power) * 0.75);
        rightFrontDrive.setPower(Math.abs(power) * 0.75);
        rightBackDrive.setPower(Math.abs(power) * 0.75);

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

    public void driveForward ( double distance, double power){
        double driveDistance = Math.abs(distance);
        drive(driveDistance, power);

    }

    public void driveBackward ( double distance, double power){
        double driveDistance = -(Math.abs(distance)); //seems redundant but does make sense, fixes user error
        drive(driveDistance, power);
    }

    public void strafe ( double distance, double power){
        int leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS); //offset
        int leftBackTarget = leftBackDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int rightBackTarget = rightBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);

        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftBackDrive.setTargetPosition(leftBackTarget);
        rightBackDrive.setTargetPosition(rightBackTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);

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

    public void strafeRight ( double distanceCm, double power){
        double strafeRightDistance = (Math.abs(distanceCm));
        strafe(strafeRightDistance, power);
    }

    public void strafeLeft ( double distanceCm, double power){
        double strafeLeftDistance = -(Math.abs(distanceCm));
        strafe(strafeLeftDistance, power);
    }
    public void spinLeftInDegrees(double degrees, double power) {
        int leftFrontTarget = leftFrontDrive.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);
        int leftBackTarget = leftBackDrive.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);
        int rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
        int rightBackTarget = rightBackDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);

        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftBackDrive.setTargetPosition(leftBackTarget);
        rightBackDrive.setTargetPosition(rightBackTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);

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

    public void spinRightInDegrees(double degrees, double power) {
        int leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
        int leftBackTarget = leftBackDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
        int rightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);
        int rightBackTarget = rightBackDrive.getCurrentPosition() - (int) (degrees * COUNTS_PER_DEGREE);

        leftFrontDrive.setTargetPosition(leftFrontTarget);
        leftBackDrive.setTargetPosition(leftBackTarget);
        rightBackDrive.setTargetPosition(rightBackTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);

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

        public void spinLeft ( double distance, double power){
            int leftFrontTarget = leftFrontDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
            int leftBackTarget = leftBackDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
            int rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
            int rightBackTarget = rightBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);

            leftFrontDrive.setTargetPosition(leftFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);

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

        public void spinRight ( double distance, double power){
            int leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
            int leftBackTarget = leftBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
            int rightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
            int rightBackTarget = rightBackDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);

            leftFrontDrive.setTargetPosition(leftFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);

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

        public void extendHangingMotor (double power) {
            hangingMotor.setPower(Math.abs(power));
        }

        public void retractHangingMotor (double power) {
            hangingMotor.setPower(-(Math.abs(power)));
        }

        private ElapsedTime timer = new ElapsedTime();

        public void timeOut (double duration) {//does nothing for specified duration
            timer.reset();
            while (timer.seconds() < duration) {
            }
        }

        public void raiseArm (double power, double duration) {
            timer.reset();
            armMotor.setPower(Math.abs(power));
            while (timer.seconds() < duration) {
                // wait for the specified duration
            }
            //arm.setPower(0.05); // Stop the arm after the duration has passed

        }

        public void lowerArm (double power, double duration) {
            timer.reset();
            armMotor.setPower(-Math.abs(power));
            while (timer.seconds() < duration) {
                // wait for the specified duration
            }
            armMotor.setPower(0); // Stop the arm after the duration has passed
        }

    public void tilt(double position, double duration) {
        tilterServo.setPosition(position); // Set the servo position
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < duration) {
            // Wait for the specified duration for the servo to reach the position
        }
    }

        public void tiltUp () {
            tilterServo.setPosition(TILTER_UP);
        }

        public void tiltMiddle () {
            tilterServo.setPosition(TILTER_MIDDLE);
        }

        public void tiltDown () {
            tilterServo.setPosition(TILTER_DOWN);
        }

        public void grab ( double position){
            grabberServo.setPosition(position);

        }

        public void openGrabber () {
            grabberServo.setPosition(GRABBER_OPEN);
        }

        public void closeGrabber () {
            grabberServo.setPosition(GRABBER_CLOSED);
        }

        public void openAutoGrabber () {
            grabberServo.setPosition(GRABBER_AUTO_OPEN);
        }

    }
