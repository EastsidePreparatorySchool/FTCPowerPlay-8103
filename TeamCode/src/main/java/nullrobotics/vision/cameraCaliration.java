package nullrobotics.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;

import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.CameraSystem;
import nullrobotics.lib.FourBarLift;

import nullrobotics.RR.drive.SampleMecanumDrive;
import nullrobotics.RR.trajectorysequence.TrajectorySequence;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvCamera;

import nullrobotics.lib.CameraSystem;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.VoidLib;

@Autonomous
public class cameraCaliration extends LinearOpMode{
    CameraSystem cams = new CameraSystem();
    calibrationPipeline cp = new calibrationPipeline();
    Point[] linePts = new Point[4];

    @Override
    public void runOpMode() {
        cams.init(hardwareMap);

        cams.TopDown.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        cams.TopDown.startStreaming(1920,1080);
                        cams.TopDown.setPipeline(cp);
                        while (1>0) {
                            telemetry.addData("camera Matrix", cp.cameraMatrix.toString());
                            telemetry.addData("distortion coeffs", cp.distCoeffs.toString());
                            telemetry.update();
                        }
                    }

                    @Override
                    public void onError(int errorCode) {
                        //if the camera could not be opened
                    }
                }
        );
        waitForStart();
    }
}
