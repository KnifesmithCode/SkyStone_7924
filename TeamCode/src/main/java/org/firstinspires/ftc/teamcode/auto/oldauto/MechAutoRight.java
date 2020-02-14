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

package org.firstinspires.ftc.teamcode.auto.oldauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Mech Auto", group = "Auto")
//@Disabled
public class MechAutoRight extends LinearOpMode {

    // Member names
    // Drive motors
    private String LEFT_MOTOR_NAME_FRONT = "leftDriveFront";
    private String LEFT_MOTOR_NAME_REAR = "leftDriveRear";
    private String RIGHT_MOTOR_NAME_FRONT = "rightDriveFront";
    private String RIGHT_MOTOR_NAME_REAR = "rightDriveRear";
    // Arm motor
    private String ARM_MOTOR_NAME = "armMotor";

    /* Declare OpMode members. */
    // Drive Motors
    private DcMotor leftDriveFront = null;
    private DcMotor leftDriveRear = null;
    private DcMotor rightDriveFront = null;
    private DcMotor rightDriveRear = null;
    // Arm
    private DcMotor armMotor = null;
    private Servo leftArmServo = null;
    private Servo rightArmServo = null;

    @Override
    public void runOpMode() {
        leftDriveFront = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME_FRONT);
        leftDriveRear = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME_REAR);
        rightDriveFront = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME_FRONT);
        rightDriveRear = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME_REAR);

        armMotor = hardwareMap.get(DcMotor.class, ARM_MOTOR_NAME);
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setDirection(DcMotor.Direction.REVERSE);

        leftArmServo.setDirection(Servo.Direction.FORWARD);
        rightArmServo.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        armMotor.setTargetPosition(840);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("MODE", "");

        armMotor.setPower(0.5d);
        telemetry.addData("POWER", "");

        sleep(2000);

        telemetry.addData("SERVOS", "");
        setServos(1.0d);
        sleep(500);
        armMotor.setPower(0d);
        telemetry.addData("POWER 0", "");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("STOP", "");

        // AndyMark encoders have 1680 encoder counts per rotation
        leftDriveFront.setTargetPosition(1680);
        leftDriveRear.setTargetPosition(1680);
        rightDriveFront.setTargetPosition(1680);
        rightDriveRear.setTargetPosition(1680);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        leftDriveFront.setPower(0.4d);
        leftDriveRear.setPower(0.4d);
        rightDriveFront.setPower(0.4d);
        rightDriveRear.setPower(0.4d);

        while(leftDriveFront.isBusy() && rightDriveFront.isBusy() &&
                leftDriveRear.isBusy() && rightDriveRear.isBusy()) {
            telemetry.addData("Drive", "left " + leftDriveFront.getCurrentPosition() + " right " +
                    rightDriveFront.getCurrentPosition());
        }

        leftDriveFront.setPower(0d);
        leftDriveRear.setPower(0d);
        rightDriveFront.setPower(0d);
        rightDriveRear.setPower(0d);

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setTargetPosition(1680);
        leftDriveRear.setTargetPosition(-1680);
        rightDriveFront.setTargetPosition(-1680);
        rightDriveRear.setTargetPosition(1680);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        leftDriveFront.setPower(0.4d);
        leftDriveRear.setPower(0.4d);
        rightDriveFront.setPower(0.4d);
        rightDriveRear.setPower(0.4d);

        while(leftDriveFront.isBusy() && rightDriveFront.isBusy() &&
                leftDriveRear.isBusy() && rightDriveRear.isBusy()) {
            telemetry.addData("Drive", "left " + leftDriveFront.getCurrentPosition() + " right " +
                    rightDriveFront.getCurrentPosition());
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     * Set BOTH servos at the same time
     */
    public void setServos(double position) {
        leftArmServo.setPosition(position);
        rightArmServo.setPosition(position);
    }

}
