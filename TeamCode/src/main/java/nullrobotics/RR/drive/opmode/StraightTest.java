package nullrobotics.RR.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import nullrobotics.RR.drive.SampleMecanumDrive;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public static double MULTIPLIER = 1.04;
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetryDash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(chassis, hardwareMap);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE*MULTIPLIER)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetryDash.addData("finalX", poseEstimate.getX());
        telemetryDash.addData("finalY", poseEstimate.getY());
        telemetryDash.addData("finalHeading", poseEstimate.getHeading());
        telemetryDash.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
