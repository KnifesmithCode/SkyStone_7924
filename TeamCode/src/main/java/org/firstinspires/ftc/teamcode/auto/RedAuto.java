package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.auto.roadrunner.drivebase.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drivebase.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.tilerunner.TilerunnerAuto;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import kotlin.Unit;

@Autonomous(name = "Red Auto", group = "RRAuto")
public class RedAuto extends LinearOpMode {
    // Enum to store the location of the SkyStone
    public enum Location {
        LEFT, CENTER, RIGHT
    }

    // Keep track of the height and width of the camera viewfinder globally
    private final static int HEIGHT = 480;
    private final static int WIDTH = 640;

    // Store the camera instance
    private OpenCvInternalCamera camera;
    // The pipeline which OpenCV will use
    private SkystonePipeline cvPipeline;

    // Bot reference for our use
    private TilerunnerAuto bot = new TilerunnerAuto();

    // Drive reference for Roadrunner use
    private SampleMecanumDriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize both references to hardware
        bot.init(hardwareMap, telemetry);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        // Tell the robot where it is located
        drive.setPoseEstimate(new Pose2d(-34, -63, -(3 * Math.PI) / 2));

        // Send status update
        telemetry.addData("Status", "Resetting encoders");
        telemetry.update();

        // Setup the horizontal motors to deploy
        bot.leftHorizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightHorizontalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.leftHorizontalMotor.setTargetPosition(1175);
        bot.rightHorizontalMotor.setTargetPosition(1212);

        bot.leftHorizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.rightHorizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Setup the vertical motors to raise the block
        bot.leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.leftVerticalMotor.setTargetPosition(200);
        bot.rightVerticalMotor.setTargetPosition(200);

        bot.leftVerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bot.rightVerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Establish instances of the phone camera and monitoring viewport
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();
        cvPipeline = new SkystonePipeline();
        camera.setFlashlightEnabled(false);
        camera.setPipeline(cvPipeline);
        camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);

        Trajectory leftTraj = drive.trajectoryBuilder()
                // Immediately start moving the horizontal slides and open the claw
                .addMarker(() -> {
                    bot.setClawServos(0.45);
                    bot.setLatchServos(0);
                    bot.leftHorizontalMotor.setPower(0.8);
                    bot.rightHorizontalMotor.setPower(0.8);
                    return Unit.INSTANCE;
                })
                // Open the grabber
                .addMarker( 1.0, () -> {
                    bot.setClawServos(0.55);
                    return Unit.INSTANCE;
                })
                // Go to the block
                .splineTo(new Pose2d(-42, -36, -(3 * Math.PI) / 2))
                .strafeTo(new Vector2d(-42, -24))
                .build();

        Trajectory centerTraj = drive.trajectoryBuilder()
                // Immediately start moving the horizontal slides and open the claw
                .addMarker(() -> {
                    bot.setClawServos(0.45);
                    bot.setLatchServos(0);
                    bot.leftHorizontalMotor.setPower(0.8);
                    bot.rightHorizontalMotor.setPower(0.8);
                    return Unit.INSTANCE;
                })
                // Open the grabber
                .addMarker( 1.0, () -> {
                    bot.setClawServos(0.55);
                    return Unit.INSTANCE;
                })
                // Go to the block
                .splineTo(new Pose2d(-36, -36, -(3 * Math.PI) / 2))
                .strafeTo(new Vector2d(-36, -24))
                .build();

        Trajectory rightTraj = drive.trajectoryBuilder()
                // Immediately start moving the horizontal slides and open the claw
                .addMarker(() -> {
                    bot.setClawServos(0.45);
                    bot.setLatchServos(0);
                    bot.leftHorizontalMotor.setPower(0.8);
                    bot.rightHorizontalMotor.setPower(0.8);
                    return Unit.INSTANCE;
                })
                // Open the grabber
                .addMarker( 1.0, () -> {
                    bot.setClawServos(0.55);
                    return Unit.INSTANCE;
                })
                // Go to the block
                .splineTo(new Pose2d(-30, -36, -(3 * Math.PI) / 2))
                .strafeTo(new Vector2d(-30, -24))
                .build();

        // Init loop
        while (!opModeIsActive() && !isStopRequested()) {
            if (cvPipeline.getLocation() != null) {
                telemetry.addData("Location", cvPipeline.getLocation().toString());
                telemetry.update();
            } else {
                telemetry.addData("Status", "Establishing pipeline");
                telemetry.update();
            }
        }

        Location location;

        waitForStart();

        // Game loop

        location = cvPipeline.getLocation();

        if(location == null) {
            location = Location.RIGHT;
        }

        switch (location) {
            case LEFT:
                telemetry.addData("Status", "Following left");
                drive.followTrajectorySync(leftTraj);
                break;
            case CENTER:
                telemetry.addData("Status", "Following center");
                drive.followTrajectorySync(centerTraj);
                break;
            case RIGHT:
                telemetry.addData("Status", "Following right");
                drive.followTrajectorySync(rightTraj);
                break;
        }
        // Grab the block
        bot.setClawServos(1.0);
        // Disable horizontal movement motors
        bot.leftHorizontalMotor.setPower(0);
        bot.rightHorizontalMotor.setPower(0);

        Trajectory moveToFoundation = drive.trajectoryBuilder()
                // Back up slightly
                .strafeTo(new Vector2d(-40, -46))
                // Get under the bridge
                .splineTo(new Pose2d(0, -46, 0))
                // Start raising the vertical slides
                .addMarker(() -> {
                    bot.leftVerticalMotor.setPower(0.8);
                    bot.rightVerticalMotor.setPower(0.8);
                    return Unit.INSTANCE;
                })
                // Go to the foundation
                .splineTo(new Pose2d(46, -52, 0))
                // Ensure we are flush to the foundation
                .build();

        drive.followTrajectorySync(moveToFoundation);

        // Turn toward the foundation
        drive.turnSync(((Math.PI) / 2) - 0.1);

        telemetry.addData("Heading", drive.getExternalHeading());
        telemetry.update();

        Trajectory grabFoundation = drive.trajectoryBuilder()
                .forward(10)
                .build();

        drive.followTrajectorySync(grabFoundation);

        // Latch onto the foundation
        bot.setLatchServos(0.55);
        // Drop the block
        bot.setClawServos(0.45);

        // Move the foundation toward the center of the field
        // so that the foundation will not hit the wall when rotated
        Trajectory moveFoundation0 = drive.trajectoryBuilder()
                .strafeLeft(18)
                .build();

        drive.followTrajectorySync(moveFoundation0);

        // Rotate the foundation
        drive.turnSync(-Math.PI / 4);

        // Move the foundation slightly toward the center again
        Trajectory moveFoundation1 = drive.trajectoryBuilder()
                .back(10)
                .build();

        drive.followTrajectorySync(moveFoundation1);

        // Finish rotating the foundation
        drive.turnSync(-(Math.PI / 4) - 0.3);

        Trajectory moveFoundation2 = drive.trajectoryBuilder()
                // Unlatch the servos
                .addMarker(() -> {
                    bot.setLatchServos(0);
                    return Unit.INSTANCE;
                })
                // Push the foundation against the wall (and into the corner)
                .forward(20)
                .build();

        drive.followTrajectorySync(moveFoundation2);

        Trajectory parkOnLine = drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0, -34, 0))
                .reverse()
                .build();

        drive.followTrajectorySync(parkOnLine);

        // Close the grabber so that the vertical slides drop smoothly
        bot.setClawServos(1.0);
    }

    static class SkystonePipeline extends OpenCvPipeline {
        // This mat will store the Cb channel, which is distance from blue
        Mat yCbCrChan2Mat = new Mat();

        public Location getLocation() {
            return location;
        }

        // This variable will store the location of the SkyStone (see enum Location)
        public Location location;

        enum Stage {
            YCbCr_CHAN2
        }

        private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             Change the input image to the YCrCb color space
             Y: Luminance (brightness)
             Cr: Red-difference (distance from pure red)
             Cb: Blue-difference (distance from pure blue)
             */
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            /* Extract only the Cb channel (blue-difference)
            Yellow is the opposite of blue, so this will make yellow dark to black, and pure blue white
            The stickers are much closer to blue than the SkyStones are, so they will appear much lighter
             */
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);

            /*
            ALL directions from the perspective of the robot
            This means that left for red is closer to the skybridge
            Left for blue is close to the wall
             */

            // X is constant (all blocks are lined up on 425 X
            // Y is specific to each block
            int x = 425;

            // Left stone
            Rect leftStoneRect = new Rect(x, 375, 5, 5);
            // Get a slice of the viewfinder
            Mat leftStone = new Mat(yCbCrChan2Mat, leftStoneRect);
            // Draw the portion sampled for debugging purposes
            Imgproc.rectangle(yCbCrChan2Mat, leftStoneRect, new Scalar(255, 255, 255), 1);
            // Calculate the average of the sampled area
            float leftStoneAvg = (float) Core.mean(leftStone).val[0];

            // Center stone
            Rect centerStoneRect = new Rect(x, 275, 5, 5);
            // Get a slice of the viewfinder
            Mat centerStone = new Mat(yCbCrChan2Mat, centerStoneRect);
            // Draw the portion sampled for debugging purposes
            Imgproc.rectangle(yCbCrChan2Mat, centerStoneRect, new Scalar(255, 255, 255), 1);
            // Calculate the average of the sampled area
            float centerStoneAvg = (float) Core.mean(centerStone).val[0];

            // Right stone
            Rect rightStoneRect = new Rect(x, 175, 5, 5);
            // Get a slice of the viewfinder
            Mat rightStone = new Mat(yCbCrChan2Mat, rightStoneRect);
            // Draw the portion sampled for debugging purposes
            Imgproc.rectangle(yCbCrChan2Mat, rightStoneRect, new Scalar(255, 255, 255), 1);
            // Calculate the average of the sampled area
            float rightStoneAvg = (float) Core.mean(rightStone).val[0];

            /*
            Compare the brightness of the blue-difference channel
            (once again, yellow should be dark, so the SkyStone should be the lightest of the three)
            By comparing the values to each other rather than a constant, lighting conditions should
            have no impact on results
             */
            if (leftStoneAvg > centerStoneAvg && leftStoneAvg > rightStoneAvg) {
                location = Location.LEFT;
            }
            if (centerStoneAvg > leftStoneAvg && centerStoneAvg > rightStoneAvg) {
                location = Location.CENTER;
            }
            if (rightStoneAvg > leftStoneAvg && rightStoneAvg > centerStoneAvg) {
                location = Location.RIGHT;
            }

            switch (stageToRenderToViewport) {
                case YCbCr_CHAN2: {
                    return yCbCrChan2Mat;
                }

                default: {
                    return input;
                }
            }
        }
    }
}