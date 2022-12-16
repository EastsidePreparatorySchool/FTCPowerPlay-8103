package nullrobotics.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;

import nullrobotics.RR.drive.NullMecanumDrive;
import nullrobotics.lib.CameraSystem;
import nullrobotics.lib.FourBarLift;

import nullrobotics.RR.trajectorysequence.TrajectorySequence;
import nullrobotics.lib.NullHardware;

@Autonomous
public class TapeDetection extends LinearOpMode {
    CameraSystem cams = new CameraSystem();
    FourBarLift fourbar = new FourBarLift();
    TapeDetectionPipeline tdp = new TapeDetectionPipeline();
    NullHardware chassis = new NullHardware();
    Point[] linePts = new Point[4];

    @Override
    public void runOpMode() {
        chassis.init(hardwareMap, telemetry);
        cams.init(hardwareMap);
        fourbar.init(hardwareMap, telemetry);
        tdp.isTapeRed = true;

        NullMecanumDrive mechdrive = new NullMecanumDrive(chassis, hardwareMap);

        cams.TopDown.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        //Usually this is where you'll want to start streaming from the camera
                        cams.TopDown.startStreaming(1920, 1080);
                        cams.TopDown.setPipeline(tdp);
                        while (true) {
                            tdp.calcPose(43, 11.25, 0, mechdrive, telemetry);
                        }
                    }

                    @Override
                    public void onError(int errorCode) {
                        //If the camera could not be opened.
                    }
                }
        );
        
        TrajectorySequence traj = mechdrive.trajectorySequenceBuilder(new Pose2d(50,12,Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(63, 12, Math.toRadians(0)), Math.toRadians(0))
                .build();
        waitForStart();
        sleep(5000);
        Pose2d currentPose = tdp.calcPose(47,11.5,0, mechdrive, telemetry);
        telemetry.addData("current Pose", currentPose.toString());
        sleep(2000);
        mechdrive.setPoseEstimate(currentPose);
        mechdrive.followTrajectorySequence(traj);
    }
}
