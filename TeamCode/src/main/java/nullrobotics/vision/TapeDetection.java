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

        fourbar.openClaw();
        fourbar.FBReachToIndex(0,1);
        fourbar.lift(275, VoidLib.LIFT_TELEOP_SPEED);

        cams.TopDown.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        //Usually this is where you'll want to start streaming from the camera
                        cams.TopDown.startStreaming(1920, 1080);
                        cams.TopDown.setPipeline(tdp);
                        while (1>0) {
                            double angleToHeading = (Math.atan((tdp.points[0].y-tdp.points[1].y)/(tdp.points[0].x-tdp.points[1].x)));
                            double distToBottomOfFrame = 3.8125+1.7501;
                            double centerOffset = 0;
                            double tapeWidth = Math.sqrt(Math.pow(tdp.points[3].x-tdp.points[0].x, 2)+Math.pow(tdp.points[3].y-tdp.points[0].y, 2));
                            double scale = 1.875/tapeWidth;
                            Point tapePos = new Point((tdp.points[0].x+tdp.points[3].x)/2, (tdp.points[0].y+tdp.points[3].y)/2);
                            double xCameraFrame = (400-tapePos.x)*scale;
                            double yCameraFrame = -(tapePos.y-450-centerOffset)*scale;
                            double distToCenterOfCamera = Math.sqrt(Math.pow(xCameraFrame, 2)+Math.pow(yCameraFrame, 2));
                            double angleCameraToTape = -1*Math.atan(yCameraFrame/xCameraFrame);
                            double angle_d = (Math.PI/2)-Math.abs(angleCameraToTape)-Math.abs(angleToHeading);
                            double cameraPosxFieldFrame= -1*Math.sin(angle_d)*distToCenterOfCamera;
                            double cameraPosyFieldFrame= -1*Math.signum(angleCameraToTape)*Math.cos(angle_d)*distToCenterOfCamera;
                            double distToCenterx = -1*Math.cos(angleToHeading)*distToBottomOfFrame;
                            double distToCentery = -1*Math.sin(angleToHeading)*distToBottomOfFrame;
                            double robotPosxFieldFrame = cameraPosxFieldFrame+distToCenterx;
                            double robotPosyFieldFrame = cameraPosyFieldFrame+distToCentery;
                            telemetry.addData("angle:", angleToHeading);
                            telemetry.addData("tape width:", tapeWidth*scale);
                            telemetry.addData("distance to center of camera:", distToCenterOfCamera);
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
                        }
                    }

                    @Override
                    public void onError(int errorCode) {
                        //If the camera could not be opened.
                    }
                }
        );
        SampleMecanumDrive mechdrive = new SampleMecanumDrive(chassis, hardwareMap);

        
        TrajectorySequence traj = mechdrive.trajectorySequenceBuilder(new Pose2d(34,10,Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(48, 9, Math.toRadians(0)), Math.toRadians(0))
                .build();
        waitForStart();
        for(int i = 0; i<20;i++) {
            linePts = tdp.points;
            if(tdp.isEmpty=false) {
                break;
            }
            sleep(50);
        }
        Pose2d currentPose = calcPose(34,10,0);
        sleep(2000);
        mechdrive.setPoseEstimate(currentPose);
        mechdrive.followTrajectorySequence(traj);
    }

    private Pose2d calcPose(double x, double y, double theta) {
        double angleToHeading = Math.atan((linePts[0].y-linePts[1].y)/(linePts[0].x-linePts[1].x));
        double distToBottomOfFrame = 3.5+1.7501;
        double centerOffset = 0;
        double tapeWidth = Math.sqrt(Math.pow(linePts[3].x-linePts[0].x, 2)+Math.pow(linePts[3].y-linePts[0].y, 2));
        double scale = 1.875/tapeWidth;
        Point tapePos = new Point((linePts[0].x+linePts[3].x)/2, (linePts[0].y+linePts[3].y)/2);
        double xCameraFrame = (800-tapePos.x)*scale;
        double yCameraFrame = -(tapePos.y-450-centerOffset)*scale;
        double distToCenterOfCamera = Math.sqrt(Math.pow(xCameraFrame, 2)+Math.pow(yCameraFrame, 2));
        double angleCameraToTape = -1*Math.atan(yCameraFrame/xCameraFrame);
        double angle_d = (Math.PI/2)-Math.abs(angleCameraToTape)-Math.abs(angleToHeading);
        double cameraPosxFieldFrame= -1*Math.sin(angle_d)*distToCenterOfCamera;
        double cameraPosyFieldFrame= -1*Math.signum(angleCameraToTape)*Math.cos(angle_d)*distToCenterOfCamera;
        double distToCenterx = -1*Math.cos(angleToHeading)*distToBottomOfFrame;
        double distToCentery = -1*Math.sin(angleToHeading)*distToBottomOfFrame;
        double robotPosxFieldFrame = cameraPosxFieldFrame+distToCenterx;
        double robotPosyFieldFrame = cameraPosyFieldFrame+distToCentery;
        telemetry.addData("angle:", Math.toDegrees(angleToHeading));
        telemetry.addData("tape width:", tapeWidth);
        telemetry.addData("distance to center of camera:", distToCenterOfCamera);
        telemetry.addData("x camera camera frame", xCameraFrame);
        telemetry.addData("y camera camera frame", yCameraFrame);
        telemetry.addData("angleCameraToTape:", Math.toDegrees(angleCameraToTape));
        telemetry.addData("x camera field frame", cameraPosxFieldFrame);
        telemetry.addData("y camera fieled frame", cameraPosyFieldFrame);
        telemetry.update();
        return new Pose2d(x+robotPosxFieldFrame, y+robotPosyFieldFrame, theta+angleToHeading);
    }
}
