package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
/*
 * This OpMode illustrates autonomous mode for driving to different wheel positions based on xPos.
 */
@Autonomous(name = "ComputerVisionAuto", group = "Concept")
//@Disabled
public class ComputerVisionAuto extends LinearOpMode {
    //Monitors
    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private double xPos = 0.0;

    //Hardware
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx arm = null;
    private DcMotorEx gripPose = null;
    private Servo grip = null;
    private IMU imu = null;

    //External monitor
    private Double yaw = 0.0;
    private Double pitch = 0.0;
    private Double roll = 0.0;

//tfod
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CenterStage.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };
    private VisionPortal visionPortal;

    //settings
    private static final int[] wheelPositionLeft = {1100, 2000, 1800, 1300}; // Example values
    private static final int[] wheelPositionCentre = {1550, 1550, 1550, 1550}; // Example values
    private static final int[] wheelPositionRight = {2000, 1100, 1300, 1800}; // Example values
    private static final int[] wheelPositionNull = {0,0,0,0};
    // Define arm and gripPose positions and gripper closed position
    private static final int armPosition = 1200; // Example value
    private static final int gripPosePosition = 280; // Example value

    private static final int armPositionUp = 1500; // Example value
    private static final int gripPosePositionUp = 280; // Example value
    private static final double gripperClosedPosition = 0.0; // Example value
    private static final double gripperOpenPosition = 0.055; // Example value

    @Override
    public void runOpMode() {

        initTfod();
        // Initialize hardware components
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "BackLeft");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "BackRight");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        gripPose = hardwareMap.get(DcMotorEx.class, "gripPose");
        grip = hardwareMap.get(Servo.class, "grip");
        imu = hardwareMap.get(IMU.class, "imu");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        gripPose.setDirection(DcMotorEx.Direction.REVERSE);

        // Set motor run modes
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gripPose.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        gripPose.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        YawPitchRollAngles robotOrientation;

        grip.setPosition(gripperOpenPosition);
        telemetry.setAutoClear(false);

        waitForStart();
        loopTime.reset();
        doTfod();
        if (opModeIsActive()) {
            // Drive to appropriate wheel positions based on xPos

            if (xPos <= 200) {
                driveToPosition(wheelPositionLeft);
            } else if (xPos <= 480) {
                driveToPosition(wheelPositionCentre);
            } else if (xPos > 480) {
                driveToPosition(wheelPositionRight);
            } else {
                driveToPosition(wheelPositionCentre);
                telemetry.addData("Vision Error", "No xPos supplied.");
                telemetry.update();
            }

            //stop the vision portal
            visionPortal.stopStreaming();

            // Set arm and gripPose positions
            arm.setTargetPosition(armPosition);
            gripPose.setTargetPosition(gripPosePosition);
            arm.setPower(0.6);
            gripPose.setPower(0.6);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Wait for arm and gripPose to reach target positions
            while (arm.isBusy() || gripPose.isBusy() || leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
                // Do nothing
                if (runtime.seconds() >= 8) {
                    break;
                }
            }

            grip.setPosition(gripperClosedPosition);
            while (runtime.seconds() < 9) {
                // do nothing
            }

            arm.setTargetPosition(armPositionUp);
            gripPose.setTargetPosition(gripPosePositionUp);
            arm.setPower(0.6);
            gripPose.setPower(0.6);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            while (runtime.seconds() < 12) {
                // do nothing
            }

            arm.setTargetPosition(0);
            gripPose.setTargetPosition(0);
            arm.setPower(1.0);
            gripPose.setPower(1.0);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            driveToPosition(wheelPositionNull);

            while (opModeIsActive()) {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
                pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
                roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

                if (yaw < -10 || yaw > 10) {
                    telemetry.addData("Heading", "Heading is over 10 degrees off!");

                }
                telemetry.addData("Powers", "%4.3f ,%4.3f, %4.3f, %4.3f", leftFrontDrive.getPower(), rightFrontDrive.getPower(), leftBackDrive.getPower(), rightBackDrive.getPower());
                telemetry.addData("Front Left Target:", leftFrontDrive.getTargetPosition());
                telemetry.addData("Front Right Target:", rightFrontDrive.getTargetPosition());
                telemetry.addData("Back Left Target:", leftBackDrive.getTargetPosition());
                telemetry.addData("Back Right Target:", rightBackDrive.getTargetPosition());
                telemetry.addData("IMU readings", "%4.1f, %4.1f, %4.1f", yaw, pitch, roll);
                telemetry.addData("Loop time:", "%4.1f", loopTime.milliseconds());
                telemetry.update();

                loopTime.reset();
            }
        }
        }




    // Method to drive to specified wheel positions
    private void driveToPosition(int[] positions) {
        ElapsedTime driveTime = new ElapsedTime();

        leftFrontDrive.setTargetPosition(positions[0]);
        rightFrontDrive.setTargetPosition(positions[1]);
        leftBackDrive.setTargetPosition(positions[2]);
        rightBackDrive.setTargetPosition(positions[3]);

        // Set motor powers and run to position
        leftFrontDrive.setPower(0.8);
        leftBackDrive.setPower(0.8);
        rightFrontDrive.setPower(0.8);
        rightBackDrive.setPower(0.8);

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);



        // Wait for motors to reach target positions
        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
             //Do nothing
//            if (yaw < -10 || yaw > 10) {
//                telemetry.addData("Heading", "Heading is over 10 degrees off!");
//
//            }
            if (driveTime.seconds() >= 8) {
                break;
            }

        }

        // Stop the motors
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {
        // Create the TensorFlow processor
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
        .build();

// Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(680, 480));
            builder.enableLiveView(true);
            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
            builder.setAutoStopLiveView(false);
            builder.addProcessor(tfod);
// Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        tfod.setClippingMargins(50, 200, 50, 10);
        tfod.setMinResultConfidence(0.75f);
    }   // end method initTfod()

    private void doTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Track the maximum size and its corresponding X position
        double maxArea = Double.MIN_VALUE;
        double xPosOfMaxArea = 0.0;

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            // Calculate area of recognition
            double area = recognition.getWidth() * recognition.getHeight();

            // Check if this recognition has a larger area than the current maximum
            if (area > maxArea) {
                maxArea = area;
                xPosOfMaxArea = x;
            }

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            telemetry.update();
        }   // end for() loop

        // Set xPos to the X position of the recognition with the largest size
        xPos = xPosOfMaxArea;
        visionPortal.stopStreaming();
    }   // end method telemetryTfod
}
