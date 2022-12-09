package nullrobotics.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.RR.drive.SampleMecanumDrive;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;

@Config
@Autonomous(group = "drive")
public class RR_Straight_Test extends LinearOpMode {

    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();

    public void runOpMode() throws InterruptedException {

        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);

        // RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SampleMecanumDrive mechdrive = new SampleMecanumDrive(hardwareMap);
        Trajectory trajectory = mechdrive.trajectoryBuilder(new Pose2d())
                .forward(72)
                .build();


        waitForStart();
        if (isStopRequested()) return;

        mechdrive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
