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

//    private final double[] armPIDF = {2.244, 0.2244, 0, 22.44};
//    private final double[] gripPIDF = {13.653, 1.3653, 0, 136.53};

    private final double[] armPIDF = {0.001, 0.0, 0.0001, 0.0};
    private final double[] gripPIDF = {0.005, 0.0, 0.0001, 0.0};
    private final int armHomePosition = 1300;
    private final int armIntakePosition = 950;
    private final int armScorePosition = 2800;
    private final int armShutdownThreshold = 5;

    private final int gripHomePosition = 270;
    private final int gripIntakePosition = 270;
    private final int gripScorePosition = 70;
    private final int gripShutdownThreshold = 5;

    private final double wheelSpeed = 0.8;
    private final double gripSpeed = 0.5;
    private final double armSpeed = 1.0;
    private final double armVelocitySet = 2920.0;
    private final double gripVelocitySet = 480.0;

    // external monitor
    private int armError = 0;
    private int gripError = 0;
    private int armLastError = 0;
    private int gripLastError = 0;
    private double armDerivative = 0.0;
    private double gripDerivative = 0.0;
    private double armIntegralSum = 0.0;
    private double gripIntegralSum = 0.0;
    private double armOut = 0.0;
    private double gripOut = 0.0;

    private double armCurrentVelocity = 0.0;
    private double gripCurrentVelocity = 0.0;
    private double maxArmVelocity = 0.0;
    private double maxGripVelocity = 0.0;

    private int armPosition = 0;
    private int gripPosition = 0;

    private boolean slowMode = false;
    private double slowMultiplier = 1.0;

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
        arm = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "arm");
        gripPose = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "gripPose");
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

        grip.setPosition(gripperOpenPosition);
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
            slowMode = gamepad1.square;

            slowMultiplier = slowMode ? 0.2 : 1.0;
            // Math out the powers for the wheelbase
            double leftFrontPower = (axial + lateral + yaw) * wheelSpeed * slowMultiplier;
            double rightFrontPower = (axial - lateral - yaw) * wheelSpeed * slowMultiplier;
            double leftBackPower = (axial - lateral + yaw) * wheelSpeed * slowMultiplier;
            double rightBackPower = (axial + lateral - yaw) * wheelSpeed * slowMultiplier;

            if (Math.abs(manualArmPower) > armManualDeadband || Math.abs(manualGripPower) > armManualDeadband) { //if powers being received are large enough
                if (!manualMode) { // if the presets are running, stop them and give the driver control
                    arm.setVelocity(0.0);
                    gripPose.setVelocity(0.0);
                    arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    gripPose.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    manualMode = true;
                }
                //assign the powers to the arm motors
                arm.setVelocity(manualArmPower * armSpeed * armVelocitySet);
                gripPose.setVelocity(manualGripPower * gripSpeed * gripVelocitySet);
            } else { //if the sticks are not being used
                if (manualMode) { // set the motors to be neutral
//                    arm.setTargetPosition(arm.getCurrentPosition());
//                    gripPose.setTargetPosition(gripPose.getCurrentPosition());
                    arm.setVelocity(0.0);
                    gripPose.setVelocity(0.0);
//                    //arm.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, armPIDFCoefficents);
//                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    //gripPose.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, gripPIDFCoefficents);
//                    gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    manualMode = false;
                }

                //preset buttons
                if (gamepad2.square) {
                    armToPosition(armHomePosition, gripHomePosition, armPIDF, gripPIDF);
                } else if (gamepad2.circle) {
                    armToPosition(armIntakePosition, gripIntakePosition, armPIDF, gripPIDF);
                } else if (gamepad2.triangle) {
                    armToPosition(armScorePosition, gripScorePosition, armPIDF, gripPIDF);
                }
            }

            //check if the preset position has been reached
//            if (!manualMode &&
//                    arm.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION &&
//                    Math.abs(arm.getTargetPosition() - arm.getCurrentPosition()) <= armShutdownThreshold &&
//                    // arm.getCurrentPosition() <= armShutdownThreshold &&
//                    gripPose.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION &&
//                    Math.abs(gripPose.getTargetPosition() - gripPose.getCurrentPosition()) <= gripShutdownThreshold
//                //&& gripPose.getCurrentPosition() <= gripShutdownThreshold
//            ) { //if preset has been reached, neutralise motors
//                arm.setPower(0.0);
//                gripPose.setPower(0.0);
//                arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                gripPose.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            }

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
            telemetry.addData("Arm velocities", "%8.2f, %8.2f", maxArmVelocity, maxGripVelocity);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm/Grip Power", "%4.2f, %4.2f", arm.getPower(), gripPose.getPower());
            telemetry.addData("Arm Position", armPosition);
            telemetry.addData("Grip Pose Position", gripPosition);
            telemetry.addData("Grip", "%4.2f", grip.getPosition());
            telemetry.addData("Loop Time", loopTime.milliseconds());
            telemetry.update();

            //reset the loop timer
            loopTime.reset();
        }
    }

    private void armToPosition(int armPositionReference, int gripPosePositionReference, double[] armPIDF, double[] gripPIDF ) {
        ElapsedTime PIDFTimer = new ElapsedTime();
        double armKp = armPIDF[0];
        double armKi = armPIDF[1];
        double armKd = armPIDF[2];
        double gripKp = gripPIDF[0];
        double gripKi = gripPIDF[1];
        double gripKd = gripPIDF[2];

        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gripPose.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        while ((Math.abs(armError) > 5 || Math.abs(gripError) > 5) && !manualMode) {
            armPosition = arm.getCurrentPosition();
            gripPosition = gripPose.getCurrentPosition();

            armError = armPositionReference - armPosition;
            gripError = gripPosePositionReference - gripPosition;

            armDerivative = (armError - armLastError) / PIDFTimer.seconds();
            gripDerivative = (gripError - gripLastError) / PIDFTimer.seconds();

            armIntegralSum = armIntegralSum + (armError * PIDFTimer.seconds());
            gripIntegralSum = gripIntegralSum + (gripError * PIDFTimer.seconds());

            armOut = (armKp * armError) + (armKi * armIntegralSum) + (armKd * armDerivative);
            gripOut = (gripKp * gripError) + (gripKi * gripIntegralSum) + (gripKd * gripDerivative);

            arm.setPower(armOut);
            gripPose.setPower(gripOut);

            armLastError = armError;
            gripLastError = gripError;

            PIDFTimer.reset();

            if (manualMode) {
                break;
            }
        }

        armPositionReference = arm.getCurrentPosition();
        gripPosePositionReference = gripPose.getCurrentPosition();
        armError = 0;
        gripError = 0;

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
