package org.firstinspires.ftc.teamcode;

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

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx arm = null;
    private DcMotorEx gripPose = null;
    private Servo grip = null;
    private IMU imu = null;

    private Double yaw = 0.0;
    private Double pitch = 0.0;
    private Double roll = 0.0;

    private double xPos = 0.0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CenterStage.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };
    private VisionPortal visionPortal;

    private static final int[] wheelPositionLeft = {1100, 2000, 1800, 1300}; // Example values
    private static final int[] wheelPositionCentre = {1550, 1550, 1550, 1550}; // Example values
    private static final int[] wheelPositionRight = {2000, 1100, 1300, 1800}; // Example values
    private static final int[] wheelPositionNull = {0,0,0,0};
    // Define arm and gripPose positions and gripper closed position
    private static final int armPosition = 210; // Example value
    private static final int gripPosePosition = 280; // Example value

    private static final int armPositionUp = 300; // Example value
    private static final int gripPosePositionUp = 280; // Example value
    private static final double gripperClosedPosition = 0.3; // Example value
    private static final double gripperOpenPosition = 0.6; // Example value

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
        imu = hardwareMap.get(IMU.class, "IMU");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
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
        robotOrientation = imu.getRobotYawPitchRollAngles();

        grip.setPosition(gripperOpenPosition);


        waitForStart();
        doTfod();
        if (opModeIsActive()) {


            // Drive to appropriate wheel positions based on xPos

            if (xPos <= 100) {
                driveToPosition(wheelPositionLeft);
            } else if (xPos <= 200) {
                driveToPosition(wheelPositionCentre);
            } else if (xPos > 200) {
                driveToPosition(wheelPositionRight);
            } else {
                driveToPosition(wheelPositionCentre);
                telemetry.addData("Vision Error", "No xPos supplied.");
            }


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
                if (runtime.seconds() >= 15) {
                    break;
                }
            }

            grip.setPosition(gripperClosedPosition);
            while (runtime.seconds() < 16) {
                // do fuck all
            }

            arm.setTargetPosition(armPositionUp);
            gripPose.setTargetPosition(gripPosePositionUp);
            arm.setPower(0.5);
            gripPose.setPower(0.5);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            while (runtime.seconds() < 20) {
                // do fuck all
            }

            driveToPosition(wheelPositionNull);
            arm.setTargetPosition(0);
            gripPose.setTargetPosition(0);
            arm.setPower(0.5);
            gripPose.setPower(0.5);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            gripPose.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            while (opModeIsActive()) {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
                pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
                roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

                if (yaw < -10 || yaw > 10) {
                    telemetry.addData("Heading", "Heading is over 10 degrees off!");

                }
                telemetry.addData("Powers", "%4.3f ,%4.3f, %4.3f, %4.3f", leftFrontDrive.getPower(), rightFrontDrive.getPower(), leftBackDrive.getPower(), rightBackDrive.getPower());
                telemetry.addData("IMU readings", "%4.1f, %4.1f, %4.1f", yaw, pitch, roll);
                telemetry.update();
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
        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Wait for motors to reach target positions
        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
            // Do nothing

            if (driveTime.seconds() >= 10) {
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

        // Create the TensorFlow processor the easy way.
        tfod = new TfodProcessor.Builder()

                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

// Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
// Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
// Choose a camera resolution. Not all cameras support all resolutions.
        //   builder.setCameraResolution(new Size(640, 480));

// Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

// Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(false);

// Set and enable the processor.
        builder.addProcessor(tfod);

// Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        tfod.setZoom(2.0);
        tfod.setMinResultConfidence(0.75f);
    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
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
        }   // end for() loop

        // Set xPos to the X position of the recognition with the largest size
        xPos = xPosOfMaxArea;
        visionPortal.stopStreaming();
    }   // end method telemetryTfod
}
