package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

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

@Autonomous(name = "Blue Auto", group = "RRAuto")
public class BlueAuto extends TunableLinearOpMode {
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

    private TilerunnerAuto bot = new TilerunnerAuto();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap, telemetry);

        // Send status update
        telemetry.addData("Status", "Resetting encoders");
        telemetry.update();

        bot.resetEncoders();

        // Establish instances of the phone camera and monitoring viewport
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.openCameraDevice();
        cvPipeline = new SkystonePipeline();
        camera.setFlashlightEnabled(false);
        camera.setPipeline(cvPipeline);
        camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);

        waitForStart();

        while(opModeIsActive()) {
            cvPipeline.setY(getInt("y"));
            telemetry.addData("Location", cvPipeline.getLocation().toString());
            telemetry.addData("Y", getInt("y"));
            telemetry.update();
        }
    }

    static class SkystonePipeline extends OpenCvPipeline {
        // This mat will store the Cb channel, which is distance from blue
        Mat yCbCrChan2Mat = new Mat();

        int y = 0;
        public void setY(int y) {
            this.y = y;
        }

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
            Rect leftStoneRect = new Rect(x, 300, 5, 5);
            Mat leftStone = new Mat(yCbCrChan2Mat, leftStoneRect);
            Imgproc.rectangle(yCbCrChan2Mat, leftStoneRect, new Scalar(255, 255, 255), 1);
            float leftStoneAvg = (float) Core.mean(leftStone).val[0];

            // Center stone
            Rect centerStoneRect = new Rect(x, 200, 5, 5);
            Mat centerStone = new Mat(yCbCrChan2Mat, centerStoneRect);
            Imgproc.rectangle(yCbCrChan2Mat, centerStoneRect, new Scalar(255, 255, 255), 1);
            float centerStoneAvg = (float) Core.mean(centerStone).val[0];

            // Right stone
            Rect rightStoneRect = new Rect(x, 100, 5, 5);
            Mat rightStone = new Mat(yCbCrChan2Mat, rightStoneRect);
            Imgproc.rectangle(yCbCrChan2Mat, rightStoneRect, new Scalar(255, 255, 255), 1);
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