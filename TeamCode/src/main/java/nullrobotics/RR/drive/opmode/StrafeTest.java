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
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetryDash = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(chassis, hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
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
