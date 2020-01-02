package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.tilerunner.TilerunnerTeleOP;

import java.util.ArrayList;

/*
 * Quick and dirty OpMode for finding the maximum velocities of motors
 */

@TeleOp(name = "Max")
public class MaxVelocityFinder extends OpMode {
    TilerunnerTeleOP bot = new TilerunnerTeleOP();
    ArrayList<Double> currentVeloc = new ArrayList<>();
    ArrayList<Double> maxVeloc = new ArrayList<>();
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx rearLeft;
    DcMotorEx rearRight;

    @Override
    public void init() {
        bot.init(hardwareMap, telemetry);
        frontLeft = (DcMotorEx) bot.leftDriveFront;
        frontRight = (DcMotorEx) bot.rightDriveFront;
        rearLeft = (DcMotorEx) bot.leftDriveRear;
        rearRight = (DcMotorEx) bot.rightDriveRear;

        for (int i = 0; i < 4; i++) {
            currentVeloc.add(0.0);
            maxVeloc.add(0.0);
        }
    }

    @Override
    public void loop() {
        bot.leftDriveFront.setPower(1.0);
        bot.leftDriveRear.setPower(1.0);
        bot.rightDriveFront.setPower(1.0);
        bot.rightDriveRear.setPower(1.0);

        currentVeloc.set(0, frontLeft.getVelocity()); // 2700
        currentVeloc.set(1, rearLeft.getVelocity()); // 2740
        currentVeloc.set(2, frontRight.getVelocity()); // 2840
        currentVeloc.set(3, rearRight.getVelocity()); // 2960

        // 2700, 2740, 2840, 2960

        for (int i = 0; i < 4; i++) {
            if (currentVeloc.get(i) > maxVeloc.get(i)) {
                maxVeloc.set(i, currentVeloc.get(i));
            }
        }

        telemetry.addData("current", currentVeloc);
        telemetry.addData("max", maxVeloc);
        telemetry.update();
    }
}
