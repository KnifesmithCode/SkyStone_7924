/* Copyright (c) 2017 FIRST. All rights reserved.
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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.tilerunner.TilerunnerAuto;
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

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "NEW Mech Auto", group = "Auto")
//@Disabled
public class NewMechAuto extends LinearOpMode {
    public enum Location {
        LEFT, CENTER, RIGHT;
    }

    private OpenCvInternalCamera phoneCam;
    private StageSwitchingPipeline stageSwitchingPipeline;

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
        bot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bot.leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        bot.leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        bot.rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        bot.rightDriveRear.setDirection(DcMotor.Direction.REVERSE);

        bot.armMotor.setDirection(DcMotor.Direction.REVERSE);

        bot.leftArmServo.setDirection(Servo.Direction.FORWARD);
        bot.rightArmServo.setDirection(Servo.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        stageSwitchingPipeline = new StageSwitchingPipeline();
        phoneCam.setFlashlightEnabled(true);
        phoneCam.setPipeline(stageSwitchingPipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        bot.armMotor.setTargetPosition(840);

        bot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bot.armMotor.setPower(0.5d);

        sleep(2000);

        bot.setClawServos(1.0d);
        sleep(500);
        bot.armMotor.setPower(0d);

        bot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // AndyMark encoders have 1680 encoder counts per rotation

        switch (stageSwitchingPipeline.getLocation()) {
            case LEFT: {
                bot.setClawServos(0.75d);
                bot.strafeLeft(250);
                bot.moveForward(2500);
                bot.setClawServos(1.0d);
                bot.moveBackward(1500);
                bot.strafeRight(8000);
                bot.setClawServos(0.75d);
                bot.moveBackward(1000);
            }
            case CENTER: {
                bot.setClawServos(0.75d);
                bot.strafeRight(500);
                bot.moveForward(2500);
                bot.setClawServos(1.0d);
                bot.moveBackward(1500);
                bot.strafeRight(8000);
                bot.setClawServos(0.75d);
                bot.moveBackward(1000);
            }
            case RIGHT: {
                bot.setClawServos(0.75d);
                bot.strafeRight(1000);
                bot.moveForward(2500);
                bot.setClawServos(1.0d);
                bot.moveBackward(1500);
                bot.strafeRight(8000);
                bot.setClawServos(0.75d);
                bot.moveBackward(1000);
            }
        }
        sleep(5000);
        //LEFT
        //CENTER
        //RIGHT
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

        private PipelineStageSwitchingExample.StageSwitchingPipeline.Stage stageToRenderToViewport = PipelineStageSwitchingExample.StageSwitchingPipeline.Stage.CUSTOM_CONTOUR_MAT;
        private PipelineStageSwitchingExample.StageSwitchingPipeline.Stage[] stages = PipelineStageSwitchingExample.StageSwitchingPipeline.Stage.values();

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
