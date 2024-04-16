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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MainDrive", group="Linear OpMode")
//@Disabled
public class MainDrive extends LinearOpMode {

    // System monitor
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime sinceStart = new ElapsedTime();
    private boolean manualMode = false;
    private boolean isEndGame = false;
    // Hardwarew
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx arm = null;
    private DcMotorEx gripPose = null;
    private Servo grip = null;
    private Servo planeLauncher = null;

    // settings
    private final double armManualDeadband = 0.03;

    private final double gripperClosedPosition = 0.0;
    private final double gripperOpenPosition = 0.055;

    private final double planeLauncherPreset = 0.3;
    private final double planeLauncherActive = 0.8;

    private final PIDFCoefficients armPIDFCoefficents = new PIDFCoefficients(0,0,0,0);
    private final PIDFCoefficients gripPIDFCoefficents = new PIDFCoefficients(0,0,0,0);
    private final int armHomePosition = 0;
    private final int armIntakePosition = 10;
    private final int armScorePosition = 600;
    private final int armShutdownThreshold = 5;

    private final int gripHomePosition = 0;
    private final int gripIntakePosition = 10;
    private final int gripScorePosition = 600;
    private final int gripShutdownThreshold = 5;

    private final double wheelSpeed = 0.8;
    private final double armSpeed = 0.5;

    // external monitor
    private double armCurrentVelocity = 0.0;
    private double gripCurrentVelocity = 0.0;
    private double maxArmVelocity = 0.0;
    private double maxGripVelocity = 0.0;

    private int armPosition = 0;
    private int gripPosition = 0;

    @Override
    public void runOpMode() {
        // declare arm power varibles
        double manualArmPower;
        double manualGripPower;

        //declare hardware
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "BackLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "BackRight");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        gripPose = hardwareMap.get(DcMotorEx.class, "gripPose");
        grip = hardwareMap.get(Servo.class, "grip");
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");

        //initialise all the hardware with correct modes
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripPose.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripPose.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        gripPose.setDirection(DcMotorEx.Direction.REVERSE);
        gripPose.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        //inform driver hub that initalisation has completed
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //start the game timers
        runtime.reset();
        sinceStart.reset();
        loopTime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //check for endgame
            if (sinceStart.seconds() > 90 && sinceStart.seconds() < 90.1) {
                isEndGame = true;
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                gamepad1.setLedColor(1, 0, 0, 30000);
                gamepad2.setLedColor(1, 0, 0, 30000);
                telemetry.addLine("Endgame has begun.");
            }

            //get all the inputs from gamepad sticks
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            manualArmPower = -gamepad2.left_stick_y;
            manualGripPower  = -gamepad2.right_stick_y;

            // Math out the powers for the wheelbase
            double leftFrontPower = (axial + lateral + yaw) * wheelSpeed;
            double rightFrontPower = (axial - lateral - yaw) * wheelSpeed;
            double leftBackPower = (axial - lateral + yaw) * wheelSpeed;
            double rightBackPower = (axial + lateral - yaw) * wheelSpeed;

            if (Math.abs(manualArmPower) > armManualDeadband || Math.abs(manualGripPower) > armManualDeadband) { //if powers being received are large enough
                if (!manualMode) { // if the presets are running, stop them and give the driver control
                    arm.setPower(0.0);
                    gripPose.setPower(0.0);
                    arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    gripPose.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    manualMode = true;
                }
                //assign the powers to the arm motors
                arm.setPower(manualArmPower * armSpeed);
                gripPose.setPower(manualGripPower * armSpeed);
            } else { //if the sticks are not being used
                if (manualMode) { // set the motors to be neutral
                    arm.setTargetPosition(arm.getCurrentPosition());
                    gripPose.setTargetPosition(gripPose.getCurrentPosition());
                    arm.setPower(0.0);
                    gripPose.setPower(0.0);
                    arm.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, armPIDFCoefficents);
                    //arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    gripPose.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, gripPIDFCoefficents);
                    //gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    manualMode = false;
                }

                //preset buttons
                if (gamepad2.square) {
                    armToPosition(armHomePosition, gripHomePosition);
                } else if (gamepad2.circle) {
                    armToPosition(armIntakePosition, gripIntakePosition);
                } else if (gamepad2.triangle) {
                    armToPosition(armScorePosition, gripScorePosition);
                }
            }

            //check if the preset position has been reached
            if (!manualMode &&
                    arm.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION &&
                    Math.abs(arm.getTargetPosition() - arm.getCurrentPosition()) <= armShutdownThreshold &&
                   // arm.getCurrentPosition() <= armShutdownThreshold &&
                    gripPose.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION &&
                    Math.abs(gripPose.getTargetPosition() - gripPose.getCurrentPosition()) <= gripShutdownThreshold
                   //&& gripPose.getCurrentPosition() <= gripShutdownThreshold
            ) { //if preset has been reached, neutralise motors
                arm.setPower(0.0);
                gripPose.setPower(0.0);
                arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                gripPose.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }

            // CENTRESTAGE plane launcher
            if (gamepad1.triangle && isEndGame) {
                planeLauncher.setPosition(planeLauncherActive);
            } else if (gamepad1.circle) {
                planeLauncher.setPosition(planeLauncherPreset);
            }

            //set grip position
            if (gamepad2.left_bumper) {
                grip.setPosition(gripperOpenPosition);
            } else if (gamepad2.right_bumper) {
                grip.setPosition(gripperClosedPosition);
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //Telemetry section
            //Get arm positions
            armPosition = arm.getCurrentPosition();
            gripPosition = gripPose.getCurrentPosition();

            //Get velocities
            velocity();
            // Send all the data back to driver hub
            telemetry.addData("Wheel velocities", "%8.2f, %8.2f", maxArmVelocity, maxGripVelocity);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm/Grip Power", "%4.2f, %4.2f", arm.getPower(), gripPose.getPower());
            telemetry.addData("Arm Position", armPosition);
            telemetry.addData("Grip Position", gripPosition);
            telemetry.addData("Grip", "%4.2f", grip.getPosition());
            telemetry.addData("Loop Time", loopTime.milliseconds());
            telemetry.update();

            //reset the loop timer
            loopTime.reset();
        }
    }

    private void armToPosition(int armPosition, int gripPosePosition) {
        arm.setTargetPosition(armPosition);
        gripPose.setTargetPosition(gripPosePosition);
        arm.setPower(armSpeed);
        gripPose.setPower(armSpeed);
        //use PIDF to go to positions
        arm.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, armPIDFCoefficents);
        //arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        gripPose.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, gripPIDFCoefficents);
        //gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    private void velocity() { // check maximum velocities
        armCurrentVelocity = arm.getVelocity();
        gripCurrentVelocity = gripPose.getVelocity();
        if (armCurrentVelocity > maxArmVelocity) {
            maxArmVelocity = armCurrentVelocity;
        }
        if (gripCurrentVelocity > maxGripVelocity) {
            maxGripVelocity = gripCurrentVelocity;
        }
    }

}
