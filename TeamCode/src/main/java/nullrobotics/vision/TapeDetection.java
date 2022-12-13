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

import org.openftc.easyopencv.OpenCvCamera;

import nullrobotics.lib.CameraSystem;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.VoidLib;

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
        tdp.color=true;

        cams.TopDown.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        //Usually this is where you'll want to start streaming from the camera
                        cams.TopDown.startStreaming(1920, 1080);
                        cams.TopDown.setPipeline(tdp);
                        /*while (1>0) {
                            double angleToHeading = (Math.atan((tdp.points[0].y-tdp.points[1].y)/(tdp.points[0].x-tdp.points[1].x)));
                            double distToBottomOfFrame = 2.5+1.7501;
                            double centerOffset = 0;
                            double tapeWidth = Math.sqrt(Math.pow(tdp.points[1].x-tdp.points[2].x, 2)+Math.pow(tdp.points[1].y-tdp.points[2].y, 2));
                            double scale = 2/tapeWidth;
                            Point tapePos = new Point((tdp.points[1].x+tdp.points[2].x)/2, (tdp.points[1].y+tdp.points[2].y)/2);
                            double xCameraFrame = (tdp.width-tapePos.x)*scale;
                            double yCameraFrame = -(tapePos.y-(tdp.height/2)-centerOffset)*scale;
                            double distToCenterOfCamera = Math.sqrt(Math.pow(xCameraFrame, 2)+Math.pow(yCameraFrame, 2));
                            double angleCameraToTape = Math.atan(yCameraFrame/xCameraFrame);
                            double angle_d = (Math.PI/2)-Math.abs(angleCameraToTape)-Math.abs(angleToHeading);
                            double cameraPosxFieldFrame= Math.sin(angle_d)*distToCenterOfCamera;
                            double cameraPosyFieldFrame= -1*Math.signum(angleCameraToTape)*Math.cos(angle_d)*distToCenterOfCamera;
                            double distToCenterx = Math.cos(angleToHeading)*distToBottomOfFrame;
                            double distToCentery = Math.sin(angleToHeading)*distToBottomOfFrame;
                            double robotPosxFieldFrame = cameraPosxFieldFrame+distToCenterx;
                            double robotPosyFieldFrame = cameraPosyFieldFrame+distToCentery;
                            telemetry.addData("angle:", angleToHeading);
                            telemetry.addData("tape width:", tapeWidth);
                            telemetry.addData("distance to center of camera:", distToCenterOfCamera);
                            telemetry.addData("frame width x", tdp.width);
                            telemetry.addData("frame width y", tdp.height);
                            telemetry.addData("x camera camera frame", xCameraFrame);
                            telemetry.addData("y camera camera frame", yCameraFrame);
                            telemetry.addData("angleCameraToTape:", Math.toDegrees(angleCameraToTape));
                            telemetry.addData("x camera field frame", cameraPosxFieldFrame);
                            telemetry.addData("y camera fieled frame", cameraPosyFieldFrame);
                            telemetry.addData("x robot center offset:", distToCenterx);
                            telemetry.addData("y robot center offset:", distToCentery);
                            telemetry.addData("x robot field frame:", robotPosxFieldFrame);
                            telemetry.addData("y robot fieled frame:", robotPosyFieldFrame);
                            telemetry.update();
                        }*/
                    }

                    @Override
                    public void onError(int errorCode) {
                        //If the camera could not be opened.
                    }
                }
        );
        SampleMecanumDrive mechdrive = new SampleMecanumDrive(chassis, hardwareMap);

        
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
