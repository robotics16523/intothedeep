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
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotMethods {
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public final double COUNTS_PER_MOTOR_REV = 537.7;
    public final double DRIVE_WHEEL_DIAMETER_CENTIMETERS = 9.6;
    double strafeTick = (COUNTS_PER_MOTOR_REV / (Math.PI * DRIVE_WHEEL_DIAMETER_CENTIMETERS));
    public final double SQUARE = 60.96; //square centimeters
    public final double DRIVE_GEAR_REDUCTION = 1.001;
    public final double DRIVE_COUNTS_PER_CENTIMETERS = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER_CENTIMETERS*Math.PI);
    public final double DRIVE_WHEEL_DIAMETER_MM = 96.0; // Diameter of the wheel
    public final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (Math.PI * DRIVE_WHEEL_DIAMETER_MM / 10); // Convert from mm to cm
    public final double WHEEL_BASE_WIDTH_CM = 2 * DRIVE_WHEEL_DIAMETER_MM / 10; // Convert from mm to cm
    public final double COUNTS_PER_DEGREE = COUNTS_PER_CM * Math.PI * WHEEL_BASE_WIDTH_CM / 360.0;
    public boolean armIsGoingDown = false;

    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

//    public void spin(double distance, double power, String direction){ TEST THIS
//        if(direction=="left"){
//            int leftfronttarget = leftFrontDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
//            int leftbacktarget = leftBackDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
//            int rightfronttarget = rightFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
//            int rightbacktarget = rightBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
//        } else if(direction=="right"){
//            int leftfronttarget = leftFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
//            int leftbacktarget = leftBackDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
//            int rightfronttarget = rightFrontDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
//            int rightbacktarget = rightBackDrive.getCurrentPosition() - (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
//        }
//
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
//    public void pivot2(double distanceCm, double power) {
//        int leftFrontTargetPosition = leftFrontDrive.getCurrentPosition() - (int) (distanceCm * DRIVE_COUNTS_PER_CENTIMETERS);
//        int leftBackTargetPosition = leftBackDrive.getCurrentPosition() - (int) (distanceCm * DRIVE_COUNTS_PER_CENTIMETERS);
//        int rightFrontTargetPosition = rightFrontDrive.getCurrentPosition() + (int) (distanceCm * DRIVE_COUNTS_PER_CENTIMETERS);
//        int rightBackTargetPosition = rightBackDrive.getCurrentPosition() + (int) (distanceCm * DRIVE_COUNTS_PER_CENTIMETERS);
//
//        leftFrontDrive.setTargetPosition(leftFrontTargetPosition);
//        leftBackDrive.setTargetPosition(leftBackTargetPosition);
//        rightFrontDrive.setTargetPosition(rightFrontTargetPosition);
//        rightBackDrive.setTargetPosition(rightBackTargetPosition);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        leftFrontDrive.setPower(Math.abs(power));
//        leftBackDrive.setPower(Math.abs(power));
//        rightFrontDrive.setPower(Math.abs(power));
//        rightBackDrive.setPower(Math.abs(power));
//        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
//        }
//    }
//
//    public void pivot(int degrees, double power) {
//        int targetPosition = (int) (degrees * COUNTS_PER_DEGREE);
//
//        // Set target positions for each motor
//        leftFrontDrive.setTargetPosition(targetPosition);
//        leftBackDrive.setTargetPosition(targetPosition);
//        rightFrontDrive.setTargetPosition(targetPosition);
//        rightBackDrive.setTargetPosition(targetPosition);
//
//        // Set motor modes to RUN_TO_POSITION
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Set power for each motor
//        leftFrontDrive.setPower(power);
//        leftBackDrive.setPower(power);
//        rightFrontDrive.setPower(power);
//        rightBackDrive.setPower(power);
//
//        // Wait until all motors reach the target position
//        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() ||
//                rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
//            // You can add other logic here if needed
//        }
//
//        // Stop all motors
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        rightBackDrive.setPower(0);
//
//        // Switch back to encoder mode for all motors
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }


//    public void rotate(double degrees, double power) {
//        double leftfrontpower = 0;
//        double rightfrontpower = 0;
//        double leftbackpower = 0;
//        double rightbackpower = 0;
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        if (degrees > 0) {
//            degrees = degrees - 12;
//            leftfrontpower = power;
//            rightfrontpower = -power;
//            leftbackpower = power;
//            rightbackpower = -power;
//        } else if (degrees < 0) {
//            degrees = degrees + 12;
//            leftfrontpower = -power;
//            rightfrontpower = power;
//            leftbackpower = -power;
//            rightbackpower = power;
//        } else {
//            leftFrontDrive.setPower(leftfrontpower);
//            rightFrontDrive.setPower(rightfrontpower);
//            leftBackDrive.setPower(leftbackpower);
//            rightBackDrive.setPower(rightbackpower);
//        }
//        while (getAngle() == 0) {
//        }
//
//        if (degrees < 0) {
//            while (getAngle() >= degrees) {
//            }
//        } else if (degrees > 0) {
//            while (getAngle() <= degrees) {
//            }
//        }
//        leftFrontDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
////        resetAngle();
//    }

//    public void quickpivot_left(){//negative 90 or 90? not sure without having robot needs test
//        double degreesNeeded = 90 - (int)getAngle();
//        pivot((int) degreesNeeded, 0.9);
//    }
//    public void quickpivot_right(){//negative 90 or 90? not sure without having robot needs test
//        double degreesNeeded = 90 - (int)getAngle();
//        pivot((int) -degreesNeeded, 0.9);
//    }
//    public void quickpivot_down(){//negative 180 or 180? not sure without having robot needs test
//        double degreesNeeded = 180 - (int)getAngle();
//        pivot((int) degreesNeeded, 0.9);
//    }
//    public void quickpivot_up(){//negative 90 or 90? not sure without having robot needs test
//        int degreesNeeded = 90 - (int)getAngle();
//        pivot(degreesNeeded, 0.9);
//    }

      public void reverseArmControls(){
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
            power  /= max;
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

    public void strafeLeft(double distanceCm, double power){
        double strafeLeftDistance = -(Math.abs(distanceCm));
        strafe(strafeLeftDistance, power);
    }

    public void drive(double distance, double power) {
        int leftfronttarget = leftFrontDrive.getCurrentPosition() + (int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
        int leftbacktarget = leftBackDrive.getCurrentPosition() +(int) (distance * DRIVE_COUNTS_PER_CENTIMETERS);
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
        drive(driveDistance,power);

    }
    public void driveBackward(double distance, double power) {
        double driveDistance = -(Math.abs(distance)); //seems redundant but does make sense, fixes user error
        drive(driveDistance,power);
    }

}
