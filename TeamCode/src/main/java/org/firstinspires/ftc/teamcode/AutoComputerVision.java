package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Autonomous(name = "ComputerVision Auto", group = "Concept")
//@Disabled
public class AutoComputerVision extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private DcMotor gripPose = null;
    private Servo grip = null;

    private double xPos = 0.0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double COUNTS_PER_INCH = (288) / (WHEEL_DIAMETER * Math.PI);

    // Define three sets of wheel positions
    private static final int[] wheelPosition1 = {1000, 1000, 1000, 1000}; // Example values
    private static final int[] wheelPosition2 = {2000, 2000, 2000, 2000}; // Example values
    private static final int[] wheelPosition3 = {3000, 3000, 3000, 3000}; // Example values

    // Define arm and gripPose positions and gripper closed position
    private static final int armPosition = 500; // Example value
    private static final int gripPosePosition = 700; // Example value
    private static final double gripperClosedPosition = 0.3; // Example value

    @Override
    public void runOpMode() {

        initTfod();
        // Initialize hardware components
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        gripPose = hardwareMap.get(DcMotor.class, "gripPose");
        grip = hardwareMap.get(Servo.class, "grip");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set motor run modes
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripPose.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripPose.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            telemetryTfod();
            // Drive to appropriate wheel positions based on xPos
            if (xPos < 100) {
                driveToPosition(wheelPosition1);
            } else if (xPos < 200) {
                driveToPosition(wheelPosition2);
            } else {
                driveToPosition(wheelPosition3);
            }

            // Set arm and gripPose positions
            arm.setTargetPosition(armPosition);
            gripPose.setTargetPosition(gripPosePosition);
            arm.setPower(1.0);
            gripPose.setPower(1.0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gripPose.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Close the gripper
            grip.setPosition(gripperClosedPosition);

            // Wait for arm and gripPose to reach target positions
            while (arm.isBusy() || gripPose.isBusy()) {
                // Do nothing
            }
        }
    }

    // Method to drive to specified wheel positions
    private void driveToPosition(int[] positions) {
        leftFrontDrive.setTargetPosition(positions[0]);
        leftBackDrive.setTargetPosition(positions[1]);
        rightFrontDrive.setTargetPosition(positions[2]);
        rightBackDrive.setTargetPosition(positions[3]);

        // Set motor powers and run to position
        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for motors to reach target positions
        while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
            // Do nothing
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
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
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
    }   // end method telemetryTfod(
}
