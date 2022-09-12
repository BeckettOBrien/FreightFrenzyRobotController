package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.vision.OpenCVElementTracker;

// TODO: Convert this and WharehouseAutoBlue to a single class that gets subclassed to change positions/rotations

@Config
@Autonomous(name = "Warehouse Red",group = "drive")
public class WarehouseAutoRed extends LinearOpMode {

    public static double SPRINT_SPEED = 0.90;
    public static int SPRINT_TIME = 1000;
    public static double ARM_SPEED = 0.45;
    public static int CLAW_MOVE_MAX_TIME = 1500;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        OpenCVElementTracker elementTracker = new OpenCVElementTracker(hardwareMap, 50);

        Pose2d loc = new Pose2d(8, -62, Math.toRadians(0));
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
                hubDropoffOffset = 18.75;
        }

        drive.setPoseEstimate(loc);
        Trajectory toShippingHub = drive.trajectoryBuilder(loc)
                .lineTo(new Vector2d(-8, -23 - hubDropoffOffset))
                .build();
        Trajectory backAway = drive.trajectoryBuilder(toShippingHub.end())
                .lineTo(new Vector2d(-8, -50))
                .build();
        Trajectory toEdge = drive.trajectoryBuilder(backAway.end())
                .lineTo(new Vector2d(8, -50))
                .build();

        robot.setArmPosition(armPosition, ARM_SPEED);
        while (robot.armIsBusy()) {
            if (isStopRequested()) return;
        }
        drive.followTrajectory(toShippingHub);
        robot.setClawPosition(Robot.CLAW_OPEN);
        sleep(CLAW_MOVE_MAX_TIME);
        drive.followTrajectory(backAway);
        robot.setArmPosition(Robot.ARM_BACK, ARM_SPEED); // TODO: Use a time marker to start moving the arm slightly after we move away from the hub
        while (robot.armIsBusy()) {
            if (isStopRequested()) return;
        }
        drive.followTrajectory(toEdge);
//        if (teamElementLoc == OpenCVElementTracker.LOCATION.UNKNOWN) { return; }
        robot.drive(1, 0, 0, SPRINT_SPEED);
        sleep(SPRINT_TIME);
        robot.brake();
    }

}
