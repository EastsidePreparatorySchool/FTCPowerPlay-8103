package nullrobotics.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

import nullrobotics.RR.drive.NullMecanumDrive;
import nullrobotics.RR.trajectorysequence.TrajectorySequence;
import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullDoc;
import nullrobotics.lib.NullHardware;
import nullrobotics.lib.CameraSystem;


@Config
//@Autonomous(name="NC One Plus Three <LG2>", group = "Auto")
public class OnePlusOne extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    AprilTagImplementation apriltgsi = new AprilTagImplementation();
    CameraSystem camsys = new CameraSystem();
//    TapeDetectionPipeline tdp = new TapeDetectionPipeline();
    NullMecanumDrive mechdrive;
    Point[] linePts = new Point[4];

    Label cornerColor;

    AprilTagDetection primaryDetection;

    boolean couldFindTag;


    @Override
    public void runOpMode() throws InterruptedException {

        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        camsys.init(hardwareMap);

        apriltgsi.init(hardwareMap, telemetry, camsys.Front);

        cornerColor = this.getCornerColor();

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

        mechdrive = new NullMecanumDrive(chassis, hardwareMap);

        //make variables
        Pose2d start;
        Pose2d tallPolePose;
        double tallPolePoseRad;
        Pose2d midPoleStack;
        double midPoleStackRad;
        Pose2d stackPickup;
        double stackPickupRadians;
        Pose2d tallPolePoseDrifted;
        double tallPolePoseDriftedRad;

        //Set internal known position
        //set positions we want to come back to

        if(getCornerColor() == Label.REDCORNER) {
            //RED CORNER
            start = new Pose2d(35.5, 62.125, Math.toRadians(90));
            tallPolePose = new Pose2d(/*31 29.25 29.75*/ 29.85, 6.75/*9 7*/, Math.toRadians(45));
            tallPolePoseRad = Math.toRadians(-100);
            midPoleStack = new Pose2d(49, 11.75, 0);
            midPoleStackRad = Math.toRadians(0);
            stackPickup = new Pose2d(62/*61.5*/, 11.75, Math.toRadians(0));
            stackPickupRadians = Math.toRadians(0);
            tallPolePoseDrifted = new Pose2d(31 /*32*/, 6.10 /*6.5 6.25*/, Math.toRadians(45));
            tallPolePoseDriftedRad = Math.toRadians(-140);
        } else if (getCornerColor() == Label.BLUECORNER) {
            //BLUE CORNER
            start = new Pose2d(-35.5, 62.125, Math.toRadians(90));
            tallPolePose = new Pose2d(-30.45/*-29.85*/, 5.7 /*6.1*/, Math.toRadians(135));
            tallPolePoseRad = Math.toRadians(-70);
            midPoleStack = new Pose2d(-49, 11.2, Math.toRadians(180));
            midPoleStackRad = Math.toRadians(185);
            stackPickup = new Pose2d(-62, 10.2 /*11.4*/, Math.toRadians(180));
            stackPickupRadians = Math.toRadians(0);
            tallPolePoseDrifted = new Pose2d(-31.5, 5.4/*6.10*/, Math.toRadians(135));
            tallPolePoseDriftedRad = Math.toRadians(-20);
        } else {
            //WTF
            start = new Pose2d(0, 0, Math.toRadians(0));
            tallPolePose = new Pose2d(0, 0, Math.toRadians(0));
            tallPolePoseRad = Math.toRadians(0);
            midPoleStack = new Pose2d(0, 0, Math.toRadians(0));
            midPoleStackRad = Math.toRadians(0);
            stackPickup = new Pose2d(0, 0, Math.toRadians(0));
            stackPickupRadians = Math.toRadians(0);
            tallPolePoseDrifted = new Pose2d(0, 0, Math.toRadians(0));
            tallPolePoseDriftedRad = Math.toRadians(0);
        }

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
                    fourbar.gotoCustomClawPos(NullDoc.CLAW_CLOSED_POS + 0.1);
//                    fourbar.reach(
//                            NullDoc.FOUR_BAR_POSITIONS_NEO[1][1] - 0.05
//                    );
                })
                .addTemporalMarker(2, () -> fourbar.lift(1250, NullDoc.LIFT_TELEOP_SPEED))
                .addTemporalMarker(3, () -> fourbar.FBReachToIndex(1, 3))
                .setReversed(true)
                .splineToLinearHeading(tallPolePose, tallPolePoseRad)
                .build();

        //Build second trajectory to the stack
        TrajectorySequence trajPoleToStack = mechdrive.trajectorySequenceBuilder(trajHomeToPole.end())
                .setReversed(false)
                .addTemporalMarker(1, () -> {
//                    fourbar.FBReachToIndex(0, 1);
                    fourbar.reach(0.70);
                })
                .splineToLinearHeading(midPoleStack,midPoleStackRad)
                .waitSeconds(0.5)
//                .addDisplacementMarker(() -> {
//                            Pose2d currentPose = tdp.calcPose(45,10.5,0, telemetry);
//                            mechdrive.setPoseEstimate(currentPose);
//                            fourbar.lift(350, NullDoc.LIFT_TELEOP_SPEED);
//                            telemetry.addData("calculated pose", currentPose.toString());
//                            telemetry.update();
//                        })
                .splineToLinearHeading(stackPickup, stackPickupRadians) //to the wall
                .build();
        TrajectorySequence trajStackToPole = mechdrive.trajectorySequenceBuilder(trajPoleToStack.end())
                .setReversed(true)
                .addTemporalMarker(1, () -> {
                    fourbar.FBReachToIndex(1, 3);
                })
                .splineToLinearHeading(tallPolePoseDrifted, tallPolePoseDriftedRad)
                .build();

        TrajectorySequence trajParkZn1Red = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(60,12,Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(-45))
                .forward(23)
                .turn(Math.toRadians(90))
                .forward(6)
                .strafeRight(6)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn2Red = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .splineToLinearHeading(new Pose2d(36,12,Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(-45))
                .forward(4)
                .turn(Math.toRadians(90))
                .forward(7)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn3Red = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(12,12,Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(135))
                .forward(20)
                .turn(Math.toRadians(-90))
                .forward(5)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();

        TrajectorySequence trajParkZn3Blue = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(60,12,Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(45))
                .forward(23)
                .turn(Math.toRadians(-90))
                .forward(6)
                .strafeLeft(2)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn2Blue = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .splineToLinearHeading(new Pose2d(36,12,Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(45))
                .forward(4)
                .turn(Math.toRadians(-90))
                .forward(7)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn1Blue = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(12,12,Math.toRadians(90)), Math.toRadians(0))
                .turn(Math.toRadians(-135))
                .forward(20)
                .turn(Math.toRadians(90))
                .forward(5)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
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
//        fourbar.FBReachToIndex(1,3);

        //Wait for start.
        waitForStart();

        fourbar.FBReachToIndex(1,3);

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
        fourbar.lift(200, NullDoc.LIFT_TELEOP_DESC_SPEED);
        fourbar.liftWaitForStop();
        sleep(1000);
        fourbar.closeClaw();
        sleep(1000);

        //Go back to the pole and place cone
        fourbar.lift(1250, NullDoc.LIFT_TELEOP_SPEED);
        mechdrive.followTrajectorySequence(trajStackToPole);
        sleep(1000);
        fourbar.FBReachToIndex(1, 2);
        sleep(500);
        fourbar.openClaw();
        //Park
        if(couldFindTag && getCornerColor() == Label.REDCORNER){
            switch (primaryDetection.id){
                case 0:
                    mechdrive.followTrajectorySequence(trajParkZn1Red);
                    break;
                case 2:
                    mechdrive.followTrajectorySequence(trajParkZn2Red);
                    break;
                case 1:
                    mechdrive.followTrajectorySequence(trajParkZn3Red);
                    break;
            }
        } else if (couldFindTag && getCornerColor() == Label.BLUECORNER) {
            switch (primaryDetection.id){
                case 0:
                    mechdrive.followTrajectorySequence(trajParkZn1Blue);
                    break;
                case 2:
                    mechdrive.followTrajectorySequence(trajParkZn2Blue);
                    break;
                case 1:
                    mechdrive.followTrajectorySequence(trajParkZn3Blue);
                    break;
            }
        }else {
            telemetry.addData("Couldn't find april tag", "Will not park.");
            telemetry.update();
        }
        fourbar.FBReachToIndex(0, 1);
        fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);

        //relay telemetry
        Pose2d poseEstimate = mechdrive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
        sleep(10000);

//        if it cant find the tag then do an extra cycle

    }

    public Label getCornerColor(){
        return Label.NONE;
    }

}