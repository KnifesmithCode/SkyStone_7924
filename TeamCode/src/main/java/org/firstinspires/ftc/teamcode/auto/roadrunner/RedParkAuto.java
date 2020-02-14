package org.firstinspires.ftc.teamcode.auto.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.drivebase.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drivebase.SampleMecanumDriveREVOptimized;

@Autonomous(name = "Red Park Auto", group = "Roadrunner")
public class RedParkAuto extends LinearOpMode {
    public static double DISTANCE = 48;
    private SampleMecanumDriveBase drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(DISTANCE)
                            .build()
            );
            drive.turnSync(Math.toRadians(90));
        }
    }
}
