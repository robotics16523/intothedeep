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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    /* Declare OpMode members. */
//    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor arm = null;
    public Servo grabber = null;
    public Servo droneLauncher = null;

    public Servo tilter = null;
    HardwareMap hardwaremap = null;
    double strafe_tick = (537.7 / (3.1415926 * 9.6));
    double forwardbackwards_tick = (537.7 / (3.1415926 * 9.6));
    public final double GRABBER_OPEN_POSITION = 0.2;
    public final double GRABBER_CLOSED_POSITION = 1.0;
    public final double DRONE_OPEN_POSITION = 0.2;//change
    public final double DRONE_CLOSED_POSITION = 1.0;

    public final double TILTER_PLACE = 0.78;//test
    public final double TILTER_PICKUP = 0.53;

    public final int ARM_MAXIMUM = -11862;//test
    public final int ARM_MINIMUM = -40;//change this?
    public final double SQUARE_LENGTH = 60.96; //centimeters

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
//        double grabberPosition = grabber.getPosition();

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

    public void launchDrone() { //test
        double droneServoPosition = 0.0;
        droneServoPosition = droneLauncher.getPosition();
        try {
            if (droneServoPosition == DRONE_CLOSED_POSITION) {
                droneLauncher.setPosition(DRONE_OPEN_POSITION);
                Thread.sleep(5000);
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
        if (arm.getCurrentPosition() >= ARM_MAXIMUM) {
            arm.setTargetPosition(ARM_MAXIMUM+100);
        }
        if (arm.getCurrentPosition() <= ARM_MINIMUM) {
            arm.setTargetPosition(ARM_MINIMUM-100);
        }
        arm.setPower(power);

       /* while (arm.isBusy()) {

        }
        if (arm.getCurrentPosition() >= ARM_MAXIMUM) {

        }
        if (arm.getCurrentPosition() <= ARM_MINIMUM) {

        }
        arm.setPower(power);*/
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
    public void sequence_attachments_a() {
       /* while (arm.isBusy()) {
        }*/
        if (arm.getCurrentPosition() < ARM_MAXIMUM) {
        }
        if (arm.getCurrentPosition() > ARM_MINIMUM) {
        }
        try {
            arm.setTargetPosition(ARM_MINIMUM - 500);
            Thread.sleep(1500);
            closeGrabber();
            Thread.sleep(200);
            tilterpickup();
            Thread.sleep(200);
            arm.setTargetPosition(ARM_MINIMUM);
            Thread.sleep(300);
            openGrabber();
            Thread.sleep(400);
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
            arm.setTargetPosition(ARM_MAXIMUM + 1000);
            Thread.sleep(1500);
            tilterplace();
            Thread.sleep(500);
            closeGrabber();
            Thread.sleep(200);
            tilterpickup();
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void strafe(double distance, double power) {
        int leftfronttarget = leftFrontDrive.getCurrentPosition() + (int) (distance * strafe_tick); //offset
        int leftbacktarget = leftBackDrive.getCurrentPosition() - (int) (distance * strafe_tick);
        int rightfronttarget = rightFrontDrive.getCurrentPosition() - (int) (distance * strafe_tick);
        int rightbacktarget = rightBackDrive.getCurrentPosition() + (int) (distance * strafe_tick);

        leftFrontDrive.setTargetPosition(leftfronttarget);
        leftBackDrive.setTargetPosition(leftbacktarget);
        rightBackDrive.setTargetPosition(rightfronttarget);
        rightFrontDrive.setTargetPosition(rightbacktarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(Math.abs(power));
        leftBackDrive.setPower(Math.abs(power));
        rightFrontDrive.setPower(Math.abs(power));
        rightBackDrive.setPower(Math.abs(power));

        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) { //invert syntax?

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

    public void drive(double distance, double power) {
        int leftfronttarget = leftFrontDrive.getCurrentPosition() + (int) (distance * forwardbackwards_tick);
        int leftbacktarget = leftBackDrive.getCurrentPosition() + (int) (distance * forwardbackwards_tick);
        int rightfronttarget = rightFrontDrive.getCurrentPosition() + (int) (distance * forwardbackwards_tick);
        int rightbacktarget = rightBackDrive.getCurrentPosition() + (int) (distance * forwardbackwards_tick);

        leftFrontDrive.setTargetPosition(leftfronttarget);
        leftBackDrive.setTargetPosition(leftbacktarget);
        rightBackDrive.setTargetPosition(rightfronttarget);
        rightFrontDrive.setTargetPosition(rightbacktarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(Math.abs(power));
        leftBackDrive.setPower(Math.abs(power));
        rightFrontDrive.setPower(Math.abs(power));
        rightBackDrive.setPower(Math.abs(power));

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
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
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
