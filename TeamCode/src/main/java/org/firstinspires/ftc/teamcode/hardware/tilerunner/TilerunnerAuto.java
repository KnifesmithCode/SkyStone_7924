package org.firstinspires.ftc.teamcode.hardware.tilerunner;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TilerunnerAuto extends TilerunnerTeleOP {
    public void strafeLeft(int counts) {
        leftDriveFront.setPower(0d);
        leftDriveRear.setPower(0d);
        rightDriveFront.setPower(0d);
        rightDriveRear.setPower(0d);

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setTargetPosition(-counts);
        leftDriveRear.setTargetPosition(counts);
        rightDriveFront.setTargetPosition(counts);
        rightDriveRear.setTargetPosition(-counts);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        leftDriveFront.setPower(0.4d);
        leftDriveRear.setPower(0.4d);
        rightDriveFront.setPower(0.4d);
        rightDriveRear.setPower(0.4d);

        while (leftDriveFront.isBusy() && rightDriveFront.isBusy() &&
                leftDriveRear.isBusy() && rightDriveRear.isBusy()) {
            telemetry.addData("Drive", "left " + leftDriveFront.getCurrentPosition() + " right " +
                    rightDriveFront.getCurrentPosition());
            telemetry.update();
        }
    }

    public void strafeRight(int counts) {
        leftDriveFront.setPower(0d);
        leftDriveRear.setPower(0d);
        rightDriveFront.setPower(0d);
        rightDriveRear.setPower(0d);

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setTargetPosition(counts);
        leftDriveRear.setTargetPosition(-counts);
        rightDriveFront.setTargetPosition(-counts);
        rightDriveRear.setTargetPosition(counts);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        leftDriveFront.setPower(0.4d);
        leftDriveRear.setPower(0.4d);
        rightDriveFront.setPower(0.4d);
        rightDriveRear.setPower(0.4d);

        while (leftDriveFront.isBusy() && rightDriveFront.isBusy() &&
                leftDriveRear.isBusy() && rightDriveRear.isBusy()) {
            telemetry.addData("Drive", "left " + leftDriveFront.getCurrentPosition() + " right " +
                    rightDriveFront.getCurrentPosition());
            telemetry.update();
        }
    }

    public void moveBackward(int counts) {

        leftDriveFront.setPower(0d);
        leftDriveRear.setPower(0d);
        rightDriveFront.setPower(0d);
        rightDriveRear.setPower(0d);

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setTargetPosition(-counts);
        leftDriveRear.setTargetPosition(-counts);
        rightDriveFront.setTargetPosition(-counts);
        rightDriveRear.setTargetPosition(-counts);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        leftDriveFront.setPower(0.4d);
        leftDriveRear.setPower(0.4d);
        rightDriveFront.setPower(0.4d);
        rightDriveRear.setPower(0.4d);

        while (leftDriveFront.isBusy() && rightDriveFront.isBusy() &&
                leftDriveRear.isBusy() && rightDriveRear.isBusy()) {
            telemetry.addData("Drive", "left " + leftDriveFront.getCurrentPosition() + " right " +
                    rightDriveFront.getCurrentPosition());
            telemetry.update();
        }
    }

    public void moveForward(int counts) {

        leftDriveFront.setPower(0d);
        leftDriveRear.setPower(0d);
        rightDriveFront.setPower(0d);
        rightDriveRear.setPower(0d);

        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setTargetPosition(counts);
        leftDriveRear.setTargetPosition(counts);
        rightDriveFront.setTargetPosition(counts);
        rightDriveRear.setTargetPosition(counts);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveRear.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        leftDriveFront.setPower(0.4d);
        leftDriveRear.setPower(0.4d);
        rightDriveFront.setPower(0.4d);
        rightDriveRear.setPower(0.4d);

        while (leftDriveFront.isBusy() && rightDriveFront.isBusy() &&
                leftDriveRear.isBusy() && rightDriveRear.isBusy()) {
            telemetry.addData("Drive", "left " + leftDriveFront.getCurrentPosition() + " right " +
                    rightDriveFront.getCurrentPosition());
            telemetry.update();
        }
    }
}
