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

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorExSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

@TeleOp(name="Mecanum Drive Test", group="Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime sinceStart = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx arm = null;
    private DcMotorEx gripPose = null;
    private Servo grip = null;
    private Servo planeLaucher = null;

    private boolean manualMode = false;
    private final double armManualDeadband = 0.03;

    private final double gripperClosedPosition = 0.3;
    private final double gripperOpenPosition = 0.5;

    private final double planeLauncherPreset = 0.3;
    private final double planeLauncherActive = 1.0;

    private final PIDFCoefficients armPIDFCoefficents = new PIDFCoefficients(0,0,0,0)
    private final PIDFCoefficients gripPIDFCoefficents = new PIDFCoefficients(0,0,0,0)
    private final int armHomePosition = 0;
    private final int armIntakePosition = 10;
    private final int armScorePosition = 600;
    private final int armShutdownThreshold = 5;

    private final int gripHomePosition = 0;
    private final int gripIntakePosition = 10;
    private final int gripScorePosition = 600;
    private final int gripShutdownThreshold = 5;

    private final double wheelSpeed = 0.6;
    private final double armSpeed = 0.5;
    private boolean isEndGame = false;

    @Override
    public void runOpMode() {
        double manualArmPower;
        double manualGripPower;
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "BackLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "BackRight");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        gripPose = hardwareMap.get(DcMotorEx.class, "gripPose");
        grip = hardwareMap.get(Servo.class, "grip");
        planeLaucher = hardwareMap.get(Servo.class, "planeLauncher");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripPose.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripPose.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        arm.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        gripPose.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        sinceStart.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (sinceStart.seconds() <= 90) {
                isEndGame = true;
            }

            if (sinceStart.seconds() == 90) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                gamepad1.setLedColor(1, 0, 0, 30000);
                gamepad2.setLedColor(1, 0, 0, 30000);
                telemetry.addLine("Endgame has begun.");
            }


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = (axial + lateral + yaw) * wheelSpeed;
            double rightFrontPower = (axial - lateral - yaw) * wheelSpeed;
            double leftBackPower = (axial - lateral + yaw) * wheelSpeed;
            double rightBackPower = (axial + lateral - yaw) * wheelSpeed;



            manualArmPower = gamepad2.right_trigger - gamepad2.left_trigger;
            manualGripPower  = gamepad2.dpad_left ? 1.0 : gamepad2.dpad_right ? 1.0 : 0.0;
          //  manualGripPower = gamepad2.right_stick_y;
            if (Math.abs(manualArmPower) > armManualDeadband || Math.abs(manualGripPower) > armManualDeadband) {
                if (!manualMode) {
                    arm.setPower(0.0);
                    gripPose.setPower(0.0);
                    arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    gripPose.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    manualMode = true;
                }
                arm.setPower(manualArmPower * armSpeed);
                gripPose.setPower(manualGripPower * armSpeed);
            } else {
                if (manualMode) {
                    arm.setTargetPosition(arm.getCurrentPosition());
                    gripPose.setTargetPosition(gripPose.getCurrentPosition());
                    arm.setPower(armSpeed);
                    gripPose.setPower(armSpeed);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    manualMode = false;
                }

                //preset buttons
                if (gamepad2.square) {
                    arm.setTargetPosition(armHomePosition);
                    gripPose.setTargetPosition(gripHomePosition);
                    arm.setPower(armSpeed);
                    gripPose.setPower(armSpeed);
                   // arm.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, new PIDFCoefficients());
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    //  grip.setPosition(wristUpPosition);
                } else if (gamepad2.circle) {
                    arm.setTargetPosition(armIntakePosition);
                    gripPose.setTargetPosition(gripIntakePosition);
                    arm.setPower(armSpeed);
                    gripPose.setPower(armSpeed);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    // grip.setPosition(wristDownPosition);
                } else if (gamepad2.triangle) {
                    arm.setTargetPosition(armScorePosition);
                    gripPose.setTargetPosition(gripScorePosition);
                    arm.setPower(armSpeed);
                    gripPose.setPower(armSpeed);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    // grip.setPosition(wristUpPosition);
                }
            }

            if (!manualMode &&
                    arm.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION &&
                    Math.abs(arm.getTargetPosition() - arm.getTargetPosition()) <= armShutdownThreshold &&
                   // arm.getCurrentPosition() <= armShutdownThreshold &&
                    gripPose.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION &&
                    Math.abs(gripPose.getTargetPosition() - gripPose.getCurrentPosition()) <= gripShutdownThreshold &&
                   // gripPose.getCurrentPosition() <= gripShutdownThreshold
            ) {
                arm.setPower(0.0);
                gripPose.setPower(0.0);
                arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                gripPose.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send all values to actuators

            if (gamepad1.triangle || isEndGame) {
                planeLaucher.setPosition(planeLauncherActive);
            } else if (gamepad1.circle) {
                planeLaucher.setPosition(planeLauncherPreset);
            }

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

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm/Grip Power", "%4.2f, %4.2f", arm.getPower(), gripPose.getPower());
            telemetry.addData("Grip", "%4.2f, %4.2f", grip.getPosition());
            telemetry.update();
        }
    }

}
