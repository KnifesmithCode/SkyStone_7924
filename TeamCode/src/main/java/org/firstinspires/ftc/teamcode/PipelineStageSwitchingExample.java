/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
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

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvPipeline#onViewportTapped()}
 * callback to switch which stage of a pipeline is rendered to the viewport for debugging
 * purposes. We also show how to get data from the pipeline to your OpMode.
 */
@TeleOp
public class PipelineStageSwitchingExample extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    StageSwitchingPipeline stageSwitchingPipeline;

    @Override
    public void runOpMode() {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCameraExample} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        stageSwitchingPipeline = new StageSwitchingPipeline();
        phoneCam.setFlashlightEnabled(true);
        phoneCam.setPipeline(stageSwitchingPipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Num contours found", stageSwitchingPipeline.getNumContoursFound());
            telemetry.addData("Current stage", stageSwitchingPipeline.getStage());
            telemetry.update();
            sleep(100);
        }
    }

    /*
     * With this pipeline, we demonstrate how to change which stage of
     * is rendered to the viewport when the viewport is tapped. This is
     * particularly useful during pipeline development. We also show how
     * to get data from the pipeline to your OpMode.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat grayscaleMat = new Mat();
        Mat grayscaleInvertMat = new Mat();
        Mat contoursOnFrameMat = new Mat();
        Mat inverseThresholdMat = new Mat();
        Mat customContourMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        List<MatOfPoint> inverseContoursList = new ArrayList<>();
        int numContoursFound;

        int thresh = 80;

        enum Stage {
            YCbCr_CHAN2,
            THRESHOLD,
            CONTOURS_OVERLAYED_ON_FRAME,
            CUSTOM_CONTOUR_MAT,
            INVERSE_THRESHOLD,
            RAW_IMAGE,
        }

        private Stage stageToRenderToViewport = Stage.CUSTOM_CONTOUR_MAT;
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
            contoursList.clear();
            inverseContoursList.clear();

            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();

            Imgproc.cvtColor(input, grayscaleMat, Imgproc.COLOR_RGB2GRAY);

            Imgproc.threshold(grayscaleMat, grayscaleInvertMat, 20, 255, Imgproc.THRESH_BINARY_INV);
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
                if (area > 5000d && area < 50000d && center.y > 400 && center.y < 500) {
                    int width = (int) (bottomRight.x - topLeft.x);
                    int height = (int) (bottomRight.y - topLeft.y);

                    Rect rect = new Rect((int) topLeft.x, (int) topLeft.y, width, height);
                    blackRects.add(rect);

                    Imgproc.circle(customContourMat, center, 3, new Scalar(0, 255, 0), 2, 8);
                    Imgproc.rectangle(customContourMat, rect, new Scalar(0, 255, 255), 2, 8);

                    // Left approx 90
                    // Center appox 200-230
                    // Right approx 290-330
                    Log.d("Found", "X: " + center.x);
                }
            }

            //Log.d("Black Rects", blackRects.toString());
            for(int i = 0; i < blackRects.size(); i++) {

            }

            for (int i = 1; i < yellowRects.size(); i++) {
                if (((yellowRects.get(i).area() - yellowRects.get(i - 1).area()) / yellowRects.get(i).area()) < 0.025d) {
                    //Imgproc.rectangle(customContourMat, yellowRects.get(i - 1), new Scalar(0, 0, 0), 5, 8);
                    //Imgproc.rectangle(customContourMat, yellowRects.get(i), new Scalar(0, 0, 0), 5, 8);
                }
            }

            switch (stageToRenderToViewport) {
                case YCbCr_CHAN2: {
                    return yCbCrChan2Mat;
                }

                case THRESHOLD: {
                    return thresholdMat;
                }

                case CONTOURS_OVERLAYED_ON_FRAME: {
                    return contoursOnFrameMat;
                }

                case CUSTOM_CONTOUR_MAT: {
                    return customContourMat;
                }

                case INVERSE_THRESHOLD: {
                    return inverseThresholdMat;
                }

                case RAW_IMAGE:

                default: {
                    return input;
                }
            }
        }

        String getStage() {
            return stageToRenderToViewport.name();
        }

        public int getNumContoursFound() {
            return numContoursFound;
        }

        public void setThreshold(int thresh) {
            this.thresh = thresh;
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