package org.firstinspires.ftc.teamcode.auto.oldauto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.tilerunner.TilerunnerAuto;
import org.firstinspires.ftc.teamcode.tuningops.PipelineStageSwitchingExampleAuto;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name = "GNU Blu!!", group = "Auto")
public class GNUMechAutoBlue extends LinearOpMode {
    public enum Location {
        LEFT, CENTER, RIGHT;
    }

    private OpenCvInternalCamera phoneCam;
    private GNUMechAutoBlue.StageSwitchingPipeline stageSwitchingPipeline;

    private TilerunnerAuto bot = new TilerunnerAuto();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        bot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        bot.leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        bot.rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        bot.rightDriveRear.setDirection(DcMotor.Direction.REVERSE);

        bot.leftArmServo.setDirection(Servo.Direction.FORWARD);
        bot.rightArmServo.setDirection(Servo.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        stageSwitchingPipeline = new GNUMechAutoBlue.StageSwitchingPipeline();
        phoneCam.setFlashlightEnabled(true);
        phoneCam.setPipeline(stageSwitchingPipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        bot.leftHorizontalMotor.setPower(0.25);
        bot.rightHorizontalMotor.setPower(0.25);

        sleep(4000);

        bot.setClawServos(0);

        sleep(1000);

        bot.setClawServos(0.75);

        sleep(1000);

        // AndyMark encoders have 1680 encoder counts per rotation

        Location l = stageSwitchingPipeline.getLocation();

        if(l == null) {
            l = Location.LEFT;
        }

        switch (l) {
            case RIGHT: {
                bot.setClawServos(0.75d);
                bot.moveForward(200);
                bot.strafeRight(250);
                bot.moveForward(3000);
                bot.setClawServos(1.0d);
                bot.moveBackward(1500);
                bot.strafeLeft(6000);
                bot.setClawServos(0.75d);
                bot.moveBackward(1000);
                bot.strafeRight(1000);
                break;
            }
            case CENTER: {
                bot.setClawServos(0.75d);
                bot.moveForward(200);
                bot.strafeLeft(500);
                bot.moveForward(3000);
                bot.setClawServos(1.0d);
                bot.moveBackward(1500);
                bot.strafeLeft(6000);
                bot.setClawServos(0.75d);
                bot.moveBackward(1000);
                bot.strafeRight(1000);
                break;
            }
            case LEFT: {
                bot.setClawServos(0.75d);
                bot.moveForward(200);
                bot.strafeLeft(1000);
                bot.moveForward(3000);
                bot.setClawServos(1.0d);
                bot.moveBackward(1500);
                bot.strafeLeft(6000);
                bot.setClawServos(0.75d);
                bot.moveBackward(1000);
                bot.strafeRight(1000);
                break;
            }
        }
        sleep(5000);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    static class StageSwitchingPipeline extends OpenCvPipeline {
        public Location l;

        public Location getLocation() {
            return l;
        }

        Mat grayscaleMat = new Mat();
        Mat grayscaleInvertMat = new Mat();
        Mat contoursOnFrameMat = new Mat();
        Mat customContourMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        List<MatOfPoint> inverseContoursList = new ArrayList<>();

        private PipelineStageSwitchingExampleAuto.StageSwitchingPipeline.Stage stageToRenderToViewport = PipelineStageSwitchingExampleAuto.StageSwitchingPipeline.Stage.CUSTOM_CONTOUR_MAT;
        private PipelineStageSwitchingExampleAuto.StageSwitchingPipeline.Stage[] stages = PipelineStageSwitchingExampleAuto.StageSwitchingPipeline.Stage.values();

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
            contoursList.clear();
            inverseContoursList.clear();

            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
            Imgproc.cvtColor(input, grayscaleMat, Imgproc.COLOR_RGB2GRAY);

            Imgproc.threshold(grayscaleMat, grayscaleInvertMat, 45, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.findContours(grayscaleInvertMat, inverseContoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            input.copyTo(customContourMat);
            //grayscaleMat.copyTo(inverseThresholdMat);

            ArrayList<Rect> yellowRects = new ArrayList<>();
            ArrayList<Rect> blackRects = new ArrayList<>();

            input.copyTo(contoursOnFrameMat);
            Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);
            for (MatOfPoint mp : contoursList) {
                if (Imgproc.contourArea(mp) > 1000d) {
                    Point topLeft = computeTopLeft(mp);
                    Point bottomRight = computeBottomRight(mp);
                    Point center = computeCenter(topLeft, bottomRight);

                    int width = (int) (bottomRight.x - topLeft.x);
                    int height = (int) (bottomRight.y - topLeft.y);

                    Rect rect = new Rect((int) topLeft.x, (int) topLeft.y, width, height);
                    yellowRects.add(rect);

                    //Imgproc.rectangle(customContourMat, new Rect((int) topLeft.x, (int) topLeft.y, width, height), new Scalar(0, 255, 0), 4);
                    //Imgproc.circle(customContourMat, center, 5, new Scalar(255, 0, 0));


                }
            }

            //Log.d("Yellow Rects", yellowRects.toString());

            for (MatOfPoint mp : inverseContoursList) {
                double area = Imgproc.contourArea(mp);
                Point topLeft = computeTopLeft(mp);
                Point bottomRight = computeBottomRight(mp);
                Point center = computeCenter(topLeft, bottomRight);
                //TODO: Tune this
                if (area > 5000d && area < 50000d && center.y > 400 && center.y < 500) {
                    int width = (int) (bottomRight.x - topLeft.x);
                    int height = (int) (bottomRight.y - topLeft.y);

                    Rect rect = new Rect((int) topLeft.x, (int) topLeft.y, width, height);
                    blackRects.add(rect);

                    Imgproc.circle(customContourMat, center, 3, new Scalar(0, 255, 0), 2, 8);
                    Imgproc.rectangle(customContourMat, rect, new Scalar(0, 255, 255), 2, 8);

                    //TODO: Tune this for competition
                    if (center.x < 120) {
                        l = Location.LEFT;
                    } else if (center.x < 250) {
                        l = Location.CENTER;
                    } else {
                        l = Location.RIGHT;
                    }
                    // Left approx 90
                    // Center appox 200-230
                    // Right approx 290-330
                    Log.d("Found", "X: " + center.x);
                }
            }

            switch (stageToRenderToViewport) {
                case CUSTOM_CONTOUR_MAT: {
                    return customContourMat;
                }
                default: {
                    return input;
                }
            }
        }

        public int[] getPixelInt(double[] pixel) {
            int[] pixelI = new int[3];
            pixelI[0] = (int) Math.round(pixel[0]);
            pixelI[1] = (int) Math.round(pixel[0]);
            pixelI[2] = (int) Math.round(pixel[0]);
            return pixelI;
        }

        public Point computeTopLeft(MatOfPoint mp) {
            Point[] list = mp.toArray();
            Point topLeft = new Point();
            for (Point p : list) {
                if (topLeft.x == 0) {
                    topLeft.x = p.x;
                }
                if (topLeft.y == 0) {
                    topLeft.y = p.y;
                }
                if (p.x < topLeft.x) {
                    topLeft.x = p.x;
                }
                if (p.y < topLeft.y) {
                    topLeft.y = p.y;
                }
            }

            return topLeft;
        }

        public Point computeBottomRight(MatOfPoint mp) {
            Point[] list = mp.toArray();
            Point bottomRight = new Point();
            for (Point p : list) {
                if (p.x > bottomRight.x) {
                    bottomRight.x = p.x;
                }
                if (p.y > bottomRight.y) {
                    bottomRight.y = p.y;
                }
            }
            return bottomRight;
        }

        public Point computeCenter(Point topLeft, Point bottomRight) {
            return new Point((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);
        }
    }
}
