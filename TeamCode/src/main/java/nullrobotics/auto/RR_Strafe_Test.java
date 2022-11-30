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
public class RR_Strafe_Test extends LinearOpMode {

    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();

    public void runOpMode() throws InterruptedException {

        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);

        // RR
        SampleMecanumDrive drive = new SampleMecanumDrive(chassis, hardwareMap);
        SampleMecanumDrive mechdrive = new SampleMecanumDrive(chassis, hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(72*1.18)
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
