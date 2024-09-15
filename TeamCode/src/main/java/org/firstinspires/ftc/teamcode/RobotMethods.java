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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class RobotHardwareMethods16523 {
    //"Every coders crime to society: set everything to null" -Felix Katch 2023
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor arm = null;
    public Servo grabber = null;
    public Servo droneLauncher = null;

    public Servo tilter = null;
    public BNO055IMU imu1 = null;
    Orientation angles;
    double globalAngle,power = 0.3,Correction;
    Orientation lastAngles = new Orientation();
    public double tpr_arm = 384.5;
    HardwareMap hardwaremap = null;
    double strafe_tick = (537.7 / (3.1415926 * 9.6));
    double forwardbackwards_tick = (537.7 / (3.1415926 * 9.6));
    public final double GRABBER_OPEN_POSITION = 0.1;
    public final double GRABBER_CLOSED_POSITION = 1.0;
    public final double DRONE_OPEN_POSITION = 0.2;//change
    public final double DRONE_CLOSED_POSITION = 1.0;

    public final double TILTER_PLACE = 0.76;//test
    public final double TILTER_PICKUP = 0.515; //test

    public final int ARM_MAXIMUM = -11862;//test
    public final int ARM_MINIMUM = 0;//change this?
    public final double SQUARE_LENGTH = 60.96; //centimeters
    public final double COUNTS_PER_MOTOR_REV = 537.7;
    public final double DRIVE_GEAR_REDUCTION = 1.0;
    public final double DRIVE_WHEEL_DIAMETER_CENTIMETERS = 9.6;
    public final double DRIVE_COUNTS_PER_CENTIMETERS = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (DRIVE_WHEEL_DIAMETER_CENTIMETERS*3.1415);
    public final double DRIVE_WHEEL_DIAMETER_MM = 96.0; // Diameter of the wheel
    public final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (Math.PI * DRIVE_WHEEL_DIAMETER_MM / 10); // Convert from mm to cm
    public final double WHEEL_BASE_WIDTH_CM = 2 * DRIVE_WHEEL_DIAMETER_MM / 10; // Convert from mm to cm
    public final double COUNTS_PER_DEGREE = COUNTS_PER_CM * Math.PI * WHEEL_BASE_WIDTH_CM / 360.0;
    private void resetAngle(){
        Orientation lastAngles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle() {
        Orientation angles = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if(deltaAngle < -180) {
            deltaAngle += 360;
        }
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grabber = hardwareMap.get(Servo.class, "grabber");
        tilter = hardwareMap.get(Servo.class, "tilter");
        droneLauncher = hardwareMap.get(Servo.class,"dronelauncher");//add to config file
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        grabber.setDirection(Servo.Direction.FORWARD);
        tilter.setDirection(Servo.Direction.FORWARD);
        droneLauncher.setDirection(Servo.Direction.FORWARD);
//        myOpMode.telemetry.addData(">", "Hardware Initialized");
//        myOpMode.telemetry.update();
    }

    public void openGrabber() {
        grabber.setPosition(GRABBER_OPEN_POSITION);
    }

    public void closeGrabber() {
        grabber.setPosition(GRABBER_CLOSED_POSITION);
    }

    public void tilterplace() {
        tilter.setPosition(TILTER_PLACE);
    }

    public void tilterpickup() {
        tilter.setPosition(TILTER_PICKUP);
    }

    public void toggleGrabber() {

        boolean isClosed = grabber.getPosition() > GRABBER_OPEN_POSITION;

        if (isClosed)
            grabber.setPosition(GRABBER_OPEN_POSITION);

        if (!isClosed)
            grabber.setPosition(GRABBER_CLOSED_POSITION);

//        if(grabberPosition  <= GRABBER_OPEN_POSITION) {
//            grabber.setPosition(GRABBER_CLOSED_POSITION);
//        } else if(grabberPosition >= GRABBER_CLOSED_POSITION && grabberPosition < GRABBER_OPEN_POSITION){
//            grabber.setPosition(GRABBER_CLOSED_POSITION);
//        } else {
//            grabber.setPosition(GRABBER_OPEN_POSITION);
//        }
    }

    public void pivot(int degrees, double power) {
        int targetPosition = (int) (degrees * COUNTS_PER_DEGREE);

        // Set target positions for each motor
        leftFrontDrive.setTargetPosition(targetPosition);
        leftBackDrive.setTargetPosition(targetPosition);
        rightFrontDrive.setTargetPosition(targetPosition);
        rightBackDrive.setTargetPosition(targetPosition);

        // Set motor modes to RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power for each motor
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);

        // Wait until all motors reach the target position
        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() ||
                rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
            // You can add other logic here if needed
        }

        // Stop all motors
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Switch back to encoder mode for all motors
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void launchDrone() { //test
        double droneServoPosition = 0.0;
        droneServoPosition = droneLauncher.getPosition();
        try {
            if (droneServoPosition == DRONE_CLOSED_POSITION) {
                droneLauncher.setPosition(DRONE_OPEN_POSITION);
                Thread.sleep(1000);//shorter duration to close the drone launcher
                droneLauncher.setPosition(DRONE_CLOSED_POSITION);
            } else {
                droneLauncher.setPosition(DRONE_CLOSED_POSITION);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return;
        }
    }

    public void moveArm(double power) {
        //if (arm.getCurrentPosition() >= ARM_MAXIMUM) {
         //   arm.setTargetPosition(ARM_MAXIMUM+100);
        //}
        //if (arm.getCurrentPosition() <= ARM_MINIMUM) {
        //    arm.setTargetPosition(ARM_MINIMUM-100);
        //}
        arm.setPower(power);

       /* while (arm.isBusy()) {

        }
        if (arm.getCurrentPosition() >= ARM_MAXIMUM) {

        }
        if (arm.getCurrentPosition() <= ARM_MINIMUM) {

        }
        arm.setPower(power);*/
    }
    public void armPosition(double target, double power){
        arm.setTargetPosition((int)(target*tpr_arm));
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(Math.abs(power));
        while(arm.isBusy()) {
        }
        if (arm.getCurrentPosition() >= ARM_MAXIMUM) {
            arm.setTargetPosition(ARM_MAXIMUM+100);
        }
        if (arm.getCurrentPosition() <= ARM_MINIMUM) {
            arm.setTargetPosition(ARM_MINIMUM-100);
        }
     //   arm.setPower(power);

    }
    //public void moveTilter(double power){
    //}
    /*
    BUTTON A:
    arm down
    tilt at 90 degrees/the floor
    grab pixel
    tilt up
    BUTTON B:
    lift the arm to desired position
     */
    public void rotate(double degrees, double power) {
        double leftfrontpower = 0;
        double rightfrontpower = 0;
        double leftbackpower = 0;
        double rightbackpower = 0;
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (degrees > 0) {
            degrees = degrees - 12;
            leftfrontpower = power;
            rightfrontpower = -power;
            leftbackpower = power;
            rightbackpower = -power;
        } else if (degrees < 0) {
            degrees = degrees + 12;
            leftfrontpower = -power;
            rightfrontpower = power;
            leftbackpower = -power;
            rightbackpower = power;
        } else {
            leftFrontDrive.setPower(leftfrontpower);
            rightFrontDrive.setPower(rightfrontpower);
            leftBackDrive.setPower(leftbackpower);
            rightBackDrive.setPower(rightbackpower);
        }
        while (getAngle() == 0) {
        }

        if (degrees < 0) {
            while (getAngle() >= degrees) {
            }
        } else if (degrees > 0) {
            while (getAngle() <= degrees) {
            }
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        resetAngle();
    }

    public void moveArmToPosition(double power, int targetPosition) {
        // Check if the target position is within the allowed range
        if (targetPosition >= ARM_MAXIMUM) {
            targetPosition = ARM_MAXIMUM;
        } else if (targetPosition <= ARM_MINIMUM) {
            targetPosition = ARM_MINIMUM;
        }

        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);

        while (arm.isBusy()) {
            // Wait until the arm reaches the target position
            // You can add other logic here if needed
        }

        arm.setPower(0); // Stop the arm
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Switch back to encoder mode
    }
    public void sequence_attachments_a() {
       /* while (arm.isBusy()) {
        }*/
        if (arm.getCurrentPosition() < ARM_MAXIMUM) {
        }
        if (arm.getCurrentPosition() > ARM_MINIMUM) {
        }
        try {
            closeGrabber();
            Thread.sleep(200);
            tilterpickup();
            Thread.sleep(200);
            armPosition(-45,.7);
            Thread.sleep(200);
            openGrabber();
            Thread.sleep(200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void sequence_attachments_b() {
       // while (arm.isBusy()) {
        //}
        if (arm.getCurrentPosition() < ARM_MAXIMUM) {
        }
        if (arm.getCurrentPosition() > ARM_MINIMUM) {
        }
        try {
            armPosition(-5000,.7);
            Thread.sleep(200);
            tilterplace();
            Thread.sleep(200);
            closeGrabber();
            Thread.sleep(200);
            tilterpickup();
            Thread.sleep(200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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
    //public void strafe_right(double distance, double power);
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

    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;
    // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;// Used to hold the data for a detected AprilTag
    /*class AprilTag {
        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();
            RobotHardwareMethods16523 robot = new RobotHardwareMethods16523();

            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }
    }*/
}
