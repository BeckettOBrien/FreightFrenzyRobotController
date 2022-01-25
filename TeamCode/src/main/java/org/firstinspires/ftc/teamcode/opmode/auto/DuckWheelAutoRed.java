package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.misc.vision.OpenCVElementTracker;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.drive.SampleMecanumDrive;

// TODO: Convert this and DuckWheelAutoRed to a single class that gets subclassed to change positions/rotations

@Config
@Autonomous(name = "Duck Wheel Red",group = "drive")
public class DuckWheelAutoRed extends LinearOpMode {

    public static double ARM_SPEED = 0.45;
    public static double DUCKWHEEL_ROTATION_OFFSET = 5;
    public static double DUCKWHEEL_SPIN_SPEED = -0.35;
    public static int ARM_MOVE_MAX_TIME = 3000;
    public static int CLAW_MOVE_MAX_TIME = 1500;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        OpenCVElementTracker elementTracker = new OpenCVElementTracker(hardwareMap);

        Pose2d loc = new Pose2d(-32, -62, Math.toRadians(0));
        OpenCVElementTracker.LOCATION teamElementLoc;
        do {
            teamElementLoc = elementTracker.getLocation();
            telemetry.addData("Current Element Location", teamElementLoc);
        } while (!isStarted());

        elementTracker.stop();
        runtime.reset();

        double hubDropoffOffset;
        int[] armPosition;
        switch (teamElementLoc) {
            case LEFT:
                armPosition = Robot.ARM_BOTTOM;
                hubDropoffOffset = 27.5;
                break;
            case RIGHT:
                armPosition = Robot.ARM_MIDDLE;
                hubDropoffOffset = 25;
                break;
            default:
                armPosition = Robot.ARM_TOP;
                hubDropoffOffset = 19.75;
        }

        drive.setPoseEstimate(loc);
        Trajectory toShippingHub = drive.trajectoryBuilder(loc)
                .lineTo(new Vector2d(-19, -24.75 - hubDropoffOffset))
                .build();
        Trajectory toDuckWheel = drive.trajectoryBuilder(toShippingHub.end())
                .lineToLinearHeading(new Pose2d(-56, -60, Math.toRadians(-90)))
                .build();
        Trajectory toStorageUnit = drive.trajectoryBuilder(toDuckWheel.end())
                .lineTo(new Vector2d(-56, -43))
                .build();

        robot.setArmPosition(armPosition, ARM_SPEED);
        while (robot.armIsBusy() && (runtime.milliseconds() < ARM_MOVE_MAX_TIME)) {
            if (isStopRequested()) return;
        }
        drive.followTrajectory(toShippingHub);
        robot.setClawPosition(Robot.CLAW_OPEN);
        sleep(CLAW_MOVE_MAX_TIME);
        robot.setArmPosition(Robot.ARM_BACK, ARM_SPEED); // TODO: Use a time marker to start moving the arm slightly after we move away from the hub
        drive.followTrajectory(toDuckWheel);
        drive.turn(Math.toRadians(DUCKWHEEL_ROTATION_OFFSET));
        robot.spinDuckWheel(DUCKWHEEL_SPIN_SPEED, 3000);
        drive.turn(Math.toRadians(-DUCKWHEEL_ROTATION_OFFSET));
        drive.followTrajectory(toStorageUnit);
    }

}
