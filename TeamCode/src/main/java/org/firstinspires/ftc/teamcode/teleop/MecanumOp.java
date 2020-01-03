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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.tilerunner.TilerunnerTeleOP;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "THE Mecanum OpMode", group = "Mecanum Control")
public class MecanumOp extends OpMode {
    private TilerunnerTeleOP bot = new TilerunnerTeleOP();

    //Keep track of runtime
    private ElapsedTime runtime = new ElapsedTime();

    // Have option for both single and two people
    private boolean singlePlayer = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        bot.init(hardwareMap, telemetry);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (gamepad1.a) {
            singlePlayer = true;
        } else if (gamepad1.b) {
            singlePlayer = false;
        }
        telemetry.addData("Single Driver", String.valueOf(singlePlayer));
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;

        double horizontalPower;

        double verticalPower;

        double clawPosition = (bot.leftArmServo.getPosition() + bot.rightArmServo.getPosition()) / 2;
        double centerClawPosition = bot.centerArmServo.getPosition();

        double latchPosition = (bot.leftLatchServo.getPosition() + bot.rightLatchServo.getPosition()) / 2;

        //#region DRIVE CONTROL
        // Utilizing a modified POV mode for mecanum transport
        // The left stick moves in straight directions, while the right stick is used to turn
        // Turning is preferred over linear motion
        // If turning is occurring, no other movement will occur
        if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
            leftFrontPower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            leftRearPower = -gamepad1.left_stick_y - gamepad1.left_stick_x;
            rightFrontPower = leftRearPower;
            rightRearPower = leftFrontPower;
        } else {
            leftFrontPower = gamepad1.right_stick_x;
            leftRearPower = leftFrontPower;
            rightFrontPower = -gamepad1.right_stick_x;
            rightRearPower = rightFrontPower;
        }
        //#endregion

        //#region VERTICAL SLIDE CONTROL
        if (singlePlayer) {
            verticalPower = gamepad1.right_trigger - (gamepad1.left_trigger / 4);
        } else {
            verticalPower = gamepad2.right_trigger - (gamepad2.left_trigger / 4);
        }
        //#endregion

        //#region HORIZONTAL SLIDE CONTROL
        if(singlePlayer) {
            horizontalPower = gamepad1.dpad_right ? 0.2 : 0;
            horizontalPower = gamepad1.dpad_left ? -0.2 : horizontalPower;
        } else {
            horizontalPower = gamepad2.dpad_right ? 0.2 : 0;
            horizontalPower = gamepad2.dpad_left ? -0.2 : horizontalPower;
        }
        //#endregion

        //#region CLAW CONTROL
        if (singlePlayer) {
            // Grabber
            if (gamepad1.a) {
                clawPosition = 1.0;
            } else if (gamepad1.x) {
                clawPosition = 0.75;
            } else if (gamepad1.b) {
                clawPosition = 0;
            } else if (gamepad1.y) {
                clawPosition = -0.5;
            }

            // Center arm
            if (gamepad1.dpad_down) {
                centerClawPosition = 0.45;
            } else if (gamepad1.dpad_up) {
                centerClawPosition = 0;
            }
        } else {
            // Grabber
            if (gamepad2.a) {
                clawPosition = 1.0;
            } else if (gamepad2.x) {
                clawPosition = 0.75;
            } else if (gamepad2.b) {
                clawPosition = 0;
            } else if (gamepad2.y) {
                clawPosition = -0.5;
            }

            // Center arm
            if (gamepad2.dpad_down) {
                centerClawPosition = 0.45;
            } else if (gamepad2.dpad_up) {
                centerClawPosition = 0;
            }
        }
        //#endregion

        //#region LATCH
        if(singlePlayer) {
            if (gamepad1.right_bumper) {
                latchPosition = 0.55;
            } else if (gamepad1.left_bumper) {
                latchPosition = 0;
            }
        } else {
            if (gamepad2.right_bumper) {
                latchPosition = 0.55;
            } else if (gamepad2.left_bumper) {
                latchPosition = 0;
            }
        }
        //#endregion

        // Set drive power
        bot.leftDriveFront.setPower(leftFrontPower);
        bot.leftDriveRear.setPower(leftRearPower);
        bot.rightDriveFront.setPower(rightFrontPower);
        bot.rightDriveRear.setPower(rightRearPower);

        // Set vertical motion power
        bot.verticalMotor.setPower(verticalPower);

        // Set horizontal motion power
        bot.leftHorizontalMotor.setPower(horizontalPower);
        bot.rightHorizontalMotor.setPower(horizontalPower);

        // Set claw servo positions
        bot.setClawServos(clawPosition);
        bot.centerArmServo.setPosition(centerClawPosition);

        // Set latch servo positions
        bot.setLatchServos(latchPosition);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Claw", "pos (%.2f)", clawPosition);
        telemetry.addData("Latch", "pos (%.2f)", latchPosition);
        telemetry.addData("Vertical", "(%.2f)", verticalPower);
        telemetry.addData("Front Motors",
                "L: (%.2f) R: (%.2f)",
                leftFrontPower, rightFrontPower);
        telemetry.addData("Rear Motors",
                "L: (%.2f) R: (%.2f)",
                leftRearPower, rightRearPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
