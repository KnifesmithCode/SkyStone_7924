package org.firstinspires.ftc.teamcode.hardware.tilerunner;

public class TilerunnerTeleOP extends Tilerunner {
    /*
     * Set BOTH claw servos at the same time
     */
    public void setClawServos(double position) {
        leftArmServo.setPosition(position);
        rightArmServo.setPosition(position);
    }

    /*
     * Set BOTH latch servos at the same time
     */
    public void setLatchServos(double position) {
        leftLatchServo.setPosition(position);
        rightLatchServo.setPosition(position + 0.05d); // Correction of 0.05 due to slight variations in servos
    }
}
