package nullrobotics.auto.RR_Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

import nullrobotics.RR.drive.SampleMecanumDrive;
import nullrobotics.RR.trajectorysequence.TrajectorySequence;
import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullHardware;
import nullrobotics.lib.CameraSystem;
import nullrobotics.vision.TapeDetectionPipeline;


@Config
@Autonomous(group = "drive")
public class Test extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    AprilTagImplementation apriltgsi = new AprilTagImplementation();
    Label signalDirection;
    CameraSystem camsys = new CameraSystem();
    TapeDetectionPipeline tdp = new TapeDetectionPipeline();
    Point[] linePts = new Point[4];


    @Override
    public void runOpMode() throws InterruptedException {

        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        camsys.init(hardwareMap);
        apriltgsi.init(hardwareMap, telemetry, camsys.Front);

        signalDirection = this.getSignalDirection();

        camsys.TopDown.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        //Usually this is where you'll want to start streaming from the camera
                        camsys.TopDown.startStreaming(1920, 1080);
                        camsys.TopDown.setPipeline(tdp);
                    }

                    @Override
                    public void onError(int errorCode) {
                        //If the camera could not be opened.
                    }
                }
        );

        SampleMecanumDrive mechdrive = new SampleMecanumDrive(chassis, hardwareMap);

        Pose2d start = new Pose2d(35.5, 62.125, Math.toRadians(90));
        ArrayList<AprilTagDetection> detections = apriltgsi.scan();
        TrajectorySequence traj = mechdrive.trajectorySequenceBuilder(start)
                .addTemporalMarker(0,() -> {
                    AprilTagDetection primaryDetection = detections.get(0);
                    //Convey back primary detection
                    apriltgsi.addDetectionToTelemetry(primaryDetection);
                })
                .setReversed(true)
                //.addTemporalMarker(0.5, () -> {fourbar.lift(1200, VoidLib.LIFT_TELEOP_SPEED);})
                .splineToLinearHeading(new Pose2d(32, 8, Math.toRadians(45)), Math.toRadians(-100))
                //.addTemporalMarker(2.75, () -> fourbar.FBReachToIndex(1, 2))
                //.addTemporalMarker(3.25, () -> fourbar.openClaw())
                .waitSeconds(1)
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    mechdrive.setPoseEstimate(calcPose(34,11,0));
                })
                .splineToLinearHeading(new Pose2d(62.5, 11, Math.toRadians(0)), Math.toRadians(0))
                /*
                .addTemporalMarker(6, () -> {
                    fourbar.FBReachToIndex(0,1);
                    fourbar.lift(225, VoidLib.LIFT_TELEOP_DESC_SPEED);
                    fourbar.closeClaw();
                })
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(32,8,Math.toRadians(45)), Math.toRadians(-140))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(62.5, 11, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(32,8,Math.toRadians(45)), Math.toRadians(-140))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(62.5, 11, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(32,8,Math.toRadians(45)), Math.toRadians(-140))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(62.5, 11, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(32,8,Math.toRadians(45)), Math.toRadians(-140))
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(62.5, 11, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(32,8,Math.toRadians(45)), Math.toRadians(-140))
                .waitSeconds(1)
                .setReversed(false)*/
                .build();
        TrajectorySequence park1 = mechdrive.trajectorySequenceBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(12,12,Math.toRadians(90)), Math.toRadians(-90))
                .build();
        TrajectorySequence park2 = mechdrive.trajectorySequenceBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(36,12,Math.toRadians(90)), Math.toRadians(90))
                .build();
        TrajectorySequence park3 = mechdrive.trajectorySequenceBuilder(traj.end())
                .splineToLinearHeading(new Pose2d(60,22,Math.toRadians(90)), Math.toRadians(45))
                .build();
        TrajectorySequence cycle5 = mechdrive.trajectorySequenceBuilder(traj.end())
                        .splineToLinearHeading(new Pose2d(62.5, 11, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(32,8,Math.toRadians(45)), Math.toRadians(-140))
                        .build();


        waitForStart();
        mechdrive.setPoseEstimate(start);
        mechdrive.followTrajectorySequence(traj);
        AprilTagDetection primaryDetection = detections.get(0);
        if(detections == null) {
            mechdrive.followTrajectorySequence(cycle5);
        } else {
            switch (primaryDetection.id) {
                case 0:
                    //Zone 1
                    mechdrive.followTrajectorySequence(park1);
                    break;
                case 2:
                    mechdrive.followTrajectorySequence(park2);
                    break;
                case 1:
                    //Zone 3
                    mechdrive.followTrajectorySequence(park3);
                    break;
            }
        }

        // RR

        /*Trajectory trajectory1 = mechdrive.trajectoryBuilder(new Pose2d())
                .forward(1.04*-50)
                .build();
        Trajectory trajectory2 = mechdrive.trajectoryBuilder(new Pose2d())
                .forward(-2*1.04)
                .build();
        Trajectory trajectory3 = mechdrive.trajectoryBuilder(new Pose2d())
                .forward(17.5*1.04)
                .build();
        Trajectory trajectory4 = mechdrive.trajectoryBuilder(new Pose2d())
                .forward(11*1.04)
                .build();
        Trajectory trajectory5 = mechdrive.trajectoryBuilder(new Pose2d())
                .forward(-19*1.04)
                .build();
        Trajectory trajectory6 = mechdrive.trajectoryBuilder(new Pose2d())
                .forward(-13*1.04)
                .build();
        int turnTwordsStack = 38;
        int turnTwordsPole = -36;
        waitForStart();
        if (isStopRequested()) return;

        //drive forward and turn twords pole
        mechdrive.followTrajectory(trajectory1);
        fourbar.FBReachToIndex(1,3);
        fourbar.lift(1200, VoidLib.LIFT_TELEOP_SPEED);
        mechdrive.turn(Math.toRadians(165)); // =45

        //raise slides and drive forward
        mechdrive.followTrajectory(trajectory2);

        //place preloaded cone and back up and drop slides
        fourbar.liftWaitForStop();
        fourbar.FBReachToIndex(1, 2);
        fourbar.openClaw();
        mechdrive.followTrajectory(trajectory3);
        fourbar.FBReachToIndex(0,1);
        fourbar.lift(450, VoidLib.LIFT_TELEOP_DESC_SPEED);

        //turn and drive to stack and grab cone 2
        mechdrive.turn(Math.toRadians(turnTwordsStack));
        mechdrive.followTrajectory(trajectory4);
        fourbar.lift(225, VoidLib.LIFT_TELEOP_DESC_SPEED);
        fourbar.liftWaitForStop();
        fourbar.closeClaw();
        sleep(300);
        fourbar.lift(1200, VoidLib.LIFT_TELEOP_SPEED);

        //go back to pole
        mechdrive.followTrajectory(trajectory5);
        fourbar.FBReachToIndex(1,3);
        mechdrive.turn(Math.toRadians(turnTwordsPole));
        mechdrive.followTrajectory(trajectory6);

        //place cone 2
        fourbar.FBReachToIndex(1,2);
        fourbar.openClaw();
        mechdrive.followTrajectory(trajectory3);
        fourbar.FBReachToIndex(0,1);
        fourbar.lift(450, VoidLib.LIFT_TELEOP_DESC_SPEED);

        //turn and drive to stack and grab cone 3
        mechdrive.turn(Math.toRadians(turnTwordsStack));
        mechdrive.followTrajectory(trajectory4);
        fourbar.lift(125, VoidLib.LIFT_TELEOP_DESC_SPEED);
        fourbar.liftWaitForStop();
        fourbar.closeClaw();
        sleep(300);
        fourbar.lift(1200, VoidLib.LIFT_TELEOP_SPEED);

        //go back to pole
        mechdrive.followTrajectory(trajectory5);
        fourbar.FBReachToIndex(1,3);
        mechdrive.turn(Math.toRadians(turnTwordsPole));
        mechdrive.followTrajectory(trajectory6);

        //place cone 3
        fourbar.FBReachToIndex(1,2);
        fourbar.openClaw();
        mechdrive.followTrajectory(trajectory3);
        fourbar.FBReachToIndex(0,1);
        fourbar.lift(450, VoidLib.LIFT_TELEOP_DESC_SPEED);

        //turn and drive to stack and grab cone 4
        mechdrive.turn(Math.toRadians(turnTwordsStack));
        mechdrive.followTrajectory(trajectory4);
        fourbar.lift(75, VoidLib.LIFT_TELEOP_DESC_SPEED);
        fourbar.liftWaitForStop();
        fourbar.closeClaw();
        sleep(300);
        fourbar.lift(1200, VoidLib.LIFT_TELEOP_SPEED);

        //go back to pole
        mechdrive.followTrajectory(trajectory5);
        fourbar.FBReachToIndex(1,3);
        mechdrive.turn(Math.toRadians(turnTwordsPole));
        mechdrive.followTrajectory(trajectory6);

        //place cone 4
        fourbar.FBReachToIndex(1,2);
        fourbar.openClaw();
        mechdrive.followTrajectory(trajectory3);
        fourbar.FBReachToIndex(0,1);
        fourbar.lift(0, VoidLib.LIFT_TELEOP_DESC_SPEED);
        /*
        //turn and drive to stack and grab cone
        mechdrive.turn(Math.toRadians(50)); // =45
        mechdrive.followTrajectory(trajectory4);
        fourbar.lift(225, VoidLib.LIFT_TELEOP_DESC_SPEED);
        fourbar.closeClaw();
        sleep(500);
        fourbar.lift(500, VoidLib.LIFT_TELEOP_SPEED);
        sleep(500);
        fourbar.FBReachToIndex(1,3);

        //go back to pole
        mechdrive.followTrajectory(trajectory5);
        mechdrive.turn(Math.toRadians(-50)); // =45
        fourbar.lift(1200, VoidLib.LIFT_TELEOP_SPEED);
        mechdrive.followTrajectory(trajectory6);

        //place cone 4
        fourbar.FBReachToIndex(1,2);
        fourbar.openClaw();
        mechdrive.followTrajectory(trajectory3);
        fourbar.FBReachToIndex(0,1);
        fourbar.lift(450, VoidLib.LIFT_TELEOP_DESC_SPEED);
        fourbar.liftWaitForStop();

        //turn and drive to stack and grab cone
        mechdrive.turn(Math.toRadians(50)); // =45
        mechdrive.followTrajectory(trajectory4);
        fourbar.lift(225, VoidLib.LIFT_TELEOP_DESC_SPEED);
        fourbar.closeClaw();
        sleep(500);
        fourbar.lift(500, VoidLib.LIFT_TELEOP_SPEED);
        sleep(500);
        fourbar.FBReachToIndex(1,3);

        //go back to pole
        mechdrive.followTrajectory(trajectory5);
        mechdrive.turn(Math.toRadians(-50)); // =45
        fourbar.lift(1200, VoidLib.LIFT_TELEOP_SPEED);
        mechdrive.followTrajectory(trajectory6);

        //place cone 5
        fourbar.FBReachToIndex(1,2);
        fourbar.openClaw();
        mechdrive.followTrajectory(trajectory3);
        fourbar.FBReachToIndex(0,1);
        fourbar.lift(450, VoidLib.LIFT_TELEOP_DESC_SPEED);
        fourbar.liftWaitForStop();
        */

        Pose2d poseEstimate = mechdrive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
    private Pose2d calcPose(double x, double y, double theta) {
        double angleToHeading = Math.atan((tdp.points[0].y-tdp.points[1].y)/(tdp.points[0].x-tdp.points[1].x));
        double distToBottomOfFrame = 3.5+1.7501;
        double centerOffset = 0;
        double tapeWidth = Math.sqrt(Math.pow(tdp.points[3].x-tdp.points[0].x, 2)+Math.pow(tdp.points[3].y-tdp.points[0].y, 2));
        double scale = 1.875/tapeWidth;
        Point tapePos = new Point((tdp.points[0].x+tdp.points[3].x)/2, (tdp.points[0].y+tdp.points[3].y)/2);
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

    public Label getSignalDirection() {
        return Label.NONE;
    }
}
