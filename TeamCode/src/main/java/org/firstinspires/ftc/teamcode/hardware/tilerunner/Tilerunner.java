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

package org.firstinspires.ftc.teamcode.hardware.tilerunner;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Defines the hardware for our old robot
 * (the one with four straight wheels and front omni-wheels)
 * <p>
 * Motor channel:  Left  drive motor:        "leftDrive"
 * Motor channel:  Right drive motor:        "rightDrive"
 * Motor channel:  Arm drive motor:          "armMotor"
 * Servo channel:  Servo to open left claw:  "leftArmServo"
 * Servo channel:  Servo to open right claw: "rightArmServo"
 */
public class Tilerunner {
    // Declare all member names
    // Drive motors
    private String LEFT_MOTOR_NAME_FRONT = "leftDriveFront";
    private String LEFT_MOTOR_NAME_REAR = "leftDriveRear";
    private String RIGHT_MOTOR_NAME_FRONT = "rightDriveFront";
    private String RIGHT_MOTOR_NAME_REAR = "rightDriveRear";
    // Arm motor
    private String ARM_MOTOR_NAME = "armMotor";
    // Claw servos
    private String LEFT_ARM_SERVO_NAME = "leftArmServo";
    private String RIGHT_ARM_SERVO_NAME = "rightArmServo";
    private String CENTER_ARM_SERVO_NAME = "centerArmServo";
    // Latch servos
    private String LEFT_LATCH_SERVO_NAME = "leftLatchServo";
    private String RIGHT_LATCH_SERVO_NAME = "rightLatchServo";

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    // Drive Motors
    public DcMotor leftDriveFront;
    public DcMotor leftDriveRear;
    public DcMotor rightDriveFront;
    public DcMotor rightDriveRear;
    // Arm
    public DcMotor armMotor;
    public Servo leftArmServo;
    public Servo rightArmServo;
    public Servo centerArmServo;
    // Latch
    public Servo leftLatchServo;
    public Servo rightLatchServo;

    // Store the hardware map
    private HardwareMap hardwareMap;

    // Store the telemetry object
    public Telemetry telemetry;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap mHardwareMap, Telemetry mTelemetry) {
        // Save reference to Hardware map
        hardwareMap = mHardwareMap;

        // Save reference to telemetry object
        telemetry = mTelemetry;

        // Get references to drive motors
        leftDriveFront = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME_FRONT);
        leftDriveRear = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME_REAR);
        rightDriveFront = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME_FRONT);
        rightDriveRear = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME_REAR);

        // Get reference to arm motor
        armMotor = hardwareMap.get(DcMotor.class, ARM_MOTOR_NAME);

        // Get reference to arm servos
        leftArmServo = hardwareMap.get(Servo.class, LEFT_ARM_SERVO_NAME);
        rightArmServo = hardwareMap.get(Servo.class, RIGHT_ARM_SERVO_NAME);
        centerArmServo = hardwareMap.get(Servo.class, CENTER_ARM_SERVO_NAME);

        // Get reference to latch servos
        leftLatchServo = hardwareMap.get(Servo.class, LEFT_LATCH_SERVO_NAME);
        rightLatchServo = hardwareMap.get(Servo.class, RIGHT_LATCH_SERVO_NAME);

        // Setup drive motors
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);

        // Setup arm motor
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        // Setup arm servos
        leftArmServo.setDirection(Servo.Direction.FORWARD);
        rightArmServo.setDirection(Servo.Direction.REVERSE);
        centerArmServo.setDirection(Servo.Direction.FORWARD);

        //Setup latch servos
        leftLatchServo.setDirection(Servo.Direction.FORWARD);
        rightLatchServo.setDirection(Servo.Direction.REVERSE);
    }
}

