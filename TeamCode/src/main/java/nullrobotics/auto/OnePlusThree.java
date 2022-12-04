package nullrobotics.auto;

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
import nullrobotics.lib.VoidLib;
import nullrobotics.vision.TapeDetectionPipeline;


@Config
@Autonomous(name="NC One Plus Three", group = "Auto")
public class OnePlusThree extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    AprilTagImplementation apriltgsi = new AprilTagImplementation();
    CameraSystem camsys = new CameraSystem();
//    TapeDetectionPipeline tdp = new TapeDetectionPipeline();
    SampleMecanumDrive mechdrive;
    Point[] linePts = new Point[4];

    AprilTagDetection primaryDetection;

    boolean couldFindTag;


    @Override
    public void runOpMode() throws InterruptedException {

        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        camsys.init(hardwareMap);

        apriltgsi.init(hardwareMap, telemetry, camsys.Front);

        //Set camera pipeline
//        camsys.TopDown.openCameraDeviceAsync(
//                new OpenCvCamera.AsyncCameraOpenListener() {
//                    @Override
//                    public void onOpened() {
//                        //Usually this is where you'll want to start streaming from the camera
//                        camsys.TopDown.startStreaming(1920, 1080);
//                        camsys.TopDown.setPipeline(tdp);
//                    }
//
//                    @Override
//                    public void onError(int errorCode) {
//                        //If the camera could not be opened.
//                    }
//                }
//        );

        mechdrive = new SampleMecanumDrive(chassis, hardwareMap);

        //Set internal known position
        Pose2d start = new Pose2d(35.5, 62.125, Math.toRadians(90));

        //set positions we want to come back to
        Pose2d tallPolePose = new Pose2d(/*31*/ 29.25, 7/*9*/, Math.toRadians(45));
        Pose2d tallPolePoseDrifted = new Pose2d(31 /*32*/, 6.5, Math.toRadians(45));


        telemetry.addData("Calculating trajectories...", "");
        telemetry.update();

        //Build first trajectory to the pole
        TrajectorySequence trajHomeToPole = mechdrive.trajectorySequenceBuilder(start)
                .addTemporalMarker(0.01,() -> {
                    //Scan for april tags
                    ArrayList<AprilTagDetection> detections = apriltgsi.scan(1000);
                    if(detections.size() != 0) {
                        couldFindTag = true;
                        this.primaryDetection = detections.get(0);
                        apriltgsi.addDetectionToTelemetry(primaryDetection);
                    } else {
                        couldFindTag = false;
                        //figure out what to do
                    }
                    fourbar.customClawPos(VoidLib.CLAW_CLOSED_POS + 0.1);
//                    fourbar.reach(
//                            VoidLib.FOUR_BAR_POSITIONS_NEO[1][1] - 0.05
//                    );
                })
                .addTemporalMarker(2, () -> fourbar.lift(1250, VoidLib.LIFT_TELEOP_SPEED))
                .addTemporalMarker(3, () -> fourbar.FBReachToIndex(1, 3))
                .setReversed(true)
                .splineToLinearHeading(tallPolePose, Math.toRadians(-100))
                .build();

        //Build second trajectory to the stack
        TrajectorySequence trajPoleToStack = mechdrive.trajectorySequenceBuilder(trajHomeToPole.end())
                .setReversed(false)
                .addTemporalMarker(1, () -> {
                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.reach(VoidLib.FOUR_BAR_POSITIONS_NEO[0][2] - 0.015);
                })
                .splineToLinearHeading(new Pose2d(49,11.75,0),Math.toRadians(0))
                .waitSeconds(0.5)
//                .addDisplacementMarker(() -> {
//                            Pose2d currentPose = tdp.calcPose(45,10.5,0, telemetry);
//                            mechdrive.setPoseEstimate(currentPose);
//                            fourbar.lift(350, VoidLib.LIFT_TELEOP_SPEED);
//                            telemetry.addData("calculated pose", currentPose.toString());
//                            telemetry.update();
//                        })
                .splineToLinearHeading(new Pose2d(61.5, 11.75, Math.toRadians(0)), Math.toRadians(0)) //to the wall
                .build();
        TrajectorySequence trajStackToPole = mechdrive.trajectorySequenceBuilder(trajPoleToStack.end())
                .setReversed(true)
                .addTemporalMarker(2, () -> {
                    fourbar.FBReachToIndex(1, 3);
                })
                .splineToLinearHeading(tallPolePoseDrifted, Math.toRadians(-140))
                .build();
        TrajectorySequence trajParkZn1 = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(60,12,Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(-45))
                .forward(23)
                .turn(Math.toRadians(90))
                .forward(6)
                .strafeRight(5)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
                    fourbar.FBReachToIndex(0, 1);
                    fourbar.lift(0, VoidLib.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn2 = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .splineToLinearHeading(new Pose2d(36,12,Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(-45))
                .forward(4)
                .turn(Math.toRadians(90))
                .forward(7)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
                    fourbar.FBReachToIndex(0, 1);
                    fourbar.lift(0, VoidLib.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn3 = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(12,12,Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(135))
                .forward(20)
                .turn(Math.toRadians(90))
                .forward(5)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
                    fourbar.FBReachToIndex(0, 1);
                    fourbar.lift(0, VoidLib.LIFT_TELEOP_DESC_SPEED);
                })
                .build();


        telemetry.addData("Done calculating trajectories.", "");
        telemetry.update();

        //grab cone
        fourbar.openClaw();
        fourbar.FBReachToIndex(1, 1);
        sleep(1000);
        fourbar.closeClaw();
        sleep(1000);
        fourbar.FBReachToIndex(1,3);

        //Wait for start.
        waitForStart();

        //Begin actual motion
        mechdrive.setPoseEstimate(start);

        //Go to pole and place cone
        mechdrive.followTrajectorySequence(trajHomeToPole);
        fourbar.liftWaitForStop();
        sleep(1000);
        fourbar.FBReachToIndex(1, 2);
        sleep(1000);
        fourbar.openClaw();
        sleep(1000);

        //Go to stack and pick up cone
        mechdrive.followTrajectorySequence(trajPoleToStack);
        fourbar.lift(200, VoidLib.LIFT_TELEOP_DESC_SPEED);
        fourbar.liftWaitForStop();
        sleep(1000);
        fourbar.closeClaw();
        sleep(1000);

        //Go back to the pole and place cone
        fourbar.lift(1250, VoidLib.LIFT_TELEOP_SPEED);
        mechdrive.followTrajectorySequence(trajStackToPole);
        sleep(1000);
        fourbar.FBReachToIndex(1, 2);
        sleep(500);
        fourbar.openClaw();
        //Park
        if(couldFindTag){
            switch (primaryDetection.id){
                case 0:
                    mechdrive.followTrajectorySequence(trajParkZn1);
                    break;
                case 2:
                    mechdrive.followTrajectorySequence(trajParkZn2);
                    break;
                case 1:
                    mechdrive.followTrajectorySequence(trajParkZn3);
                    break;
            }
        } else {
            telemetry.addData("Couldn't find april tag", "Will not park.");
            telemetry.update();
        }


        //relay telemetry
        Pose2d poseEstimate = mechdrive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        sleep(10000);

//        if it cant find the tag then do an extra cycle

    }

}
