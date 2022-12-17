package nullrobotics.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

import nullrobotics.RR.drive.NullMecanumDrive;
import nullrobotics.RR.trajectorysequence.TrajectorySequence;
import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullDoc;
import nullrobotics.lib.NullHardware;
import nullrobotics.lib.CameraSystem;
import nullrobotics.vision.TapeDetectionPipeline;


//@Autonomous
public class OnePlusFour extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    AprilTagImplementation apriltgsi = new AprilTagImplementation();
    CameraSystem camsys = new CameraSystem();
    TapeDetectionPipeline tdp = new TapeDetectionPipeline();
    NullMecanumDrive mechdrive;
    Point[] linePts = new Point[4];

    Label cornerColor;
    Label allianceColor;

    AprilTagDetection primaryDetection;

    boolean couldFindTag;

    int numStackCycles = 3;

    @Override
    public void runOpMode() throws InterruptedException {

//        chassis.init(hardwareMap, telemetry);
//        fourbar.init(hardwareMap, telemetry);
//        camsys.init(hardwareMap);

        chassis.init(hardwareMap, telemetry);
        camsys.init(hardwareMap);
        apriltgsi.init(hardwareMap, telemetry, camsys.Front);

        cornerColor = this.getCornerColor();
        allianceColor = this.getAllianceColor();

        //Set camera pipeline
        camsys.TopDown.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        //Usually this is where you'll want to start streaming from the camera
                        camsys.TopDown.startStreaming(1920, 1080);
                    }

                    @Override
                    public void onError(int errorCode) {
                        //If the camera could not be opened.
                    }
                }
        );

        camsys.TopDown.setPipeline(tdp);

        //Mechdrive for roadrunner
        mechdrive = new NullMecanumDrive(chassis, hardwareMap);

        //make variables
        Pose2d start;
        Pose2d tallPolePose;
        double tallPolePoseRad;
        Pose2d tapeFirstPose;
        double tapeFirstPoseRad;
        Pose2d stackPickup;
        double stackPickupRadians;
        Pose2d tallPolePoseDrifted;
        double tallPolePoseDriftedRad;
        Pose2d tapeDrifted;
        double tapeDriftedRad;
        double inputx;
        double inputy;
        double inputTheta;
        Pose2d stackPosRedef;

        //Set internal known position
        //set positions we want to come back to
//        start = new Pose2d(35.5, 62.125, Math.toRadians(90));
//        tallPolePose = new Pose2d(31.25, 7.25, Math.toRadians(45));
//        tallPolePoseRad = Math.toRadians(-100);
//        tapeFirstPose = new Pose2d(51, 10.5, 0);
//        tapeFirstPoseRad = Math.toRadians(0);
//        stackPickup = new Pose2d(61.75, 11.75, Math.toRadians(0));
//        stackPickupRadians = Math.toRadians(0);
//        tallPolePoseDrifted = new Pose2d(30, 4 , Math.toRadians(45));
//        tallPolePoseDriftedRad = Math.toRadians(-140);
//        tapeDrifted = new Pose2d(51.5,9,0);
//        tapeDriftedRad = Math.toRadians(0);
       if(cornerColor == Label.REDCORNER) {
            //RED CORNER
           start = new Pose2d(35.5, 62.125, Math.toRadians(90));
           tallPolePose = new Pose2d(31.75, /*6.75*/7.75, Math.toRadians(45));
           tallPolePoseRad = Math.toRadians(-100);
           tapeFirstPose = new Pose2d(/*52*/51.5, 11, 0);
           tapeFirstPoseRad = Math.toRadians(0);
           stackPickup = new Pose2d(61.75, 11.75, Math.toRadians(0));
           stackPickupRadians = Math.toRadians(0);
           tallPolePoseDrifted = new Pose2d(31, 5.5 , Math.toRadians(45));
           tallPolePoseDriftedRad = Math.toRadians(-140);
           tapeDrifted = new Pose2d(51.5,9,0);
           tapeDriftedRad = Math.toRadians(0);
           inputx = 45;
           inputy = 11.5;
           inputTheta = Math.toRadians(0);
           stackPosRedef = new Pose2d(62.75, 11.75, Math.toRadians(0));
        } else if (cornerColor == Label.BLUECORNER) {
            //BLUE CORNER
            start = new Pose2d(-35.5, 62.125, Math.toRadians(90));
            tallPolePose = new Pose2d(-32.75, 7.75, Math.toRadians(135));
            tallPolePoseRad = Math.toRadians(-62); //-70
            tapeFirstPose = new Pose2d(-53, 11.75, Math.toRadians(180));
            tapeFirstPoseRad = Math.toRadians(185);
            stackPickup = new Pose2d(-61.75, 11.75 , Math.toRadians(180));
            stackPickupRadians = Math.toRadians(0);
            tallPolePoseDrifted = new Pose2d(/*-31*/ -32, /*4*/6, Math.toRadians(135));
            tallPolePoseDriftedRad = Math.toRadians(-20);
            tapeDrifted = new Pose2d(-51.5,11 /*11*/,Math.toRadians(180));
            tapeDriftedRad = Math.toRadians(185);
            inputx = -45;
            inputy = 11.5;
            inputTheta = Math.toRadians(180);
            stackPosRedef = new Pose2d(-62.75, 11.75, Math.toRadians(180));
        } else {
            //WTF
            start = new Pose2d(0, 0, Math.toRadians(0));
            tallPolePose = new Pose2d(0, 0, Math.toRadians(0));
            tallPolePoseRad = Math.toRadians(0);
            tapeFirstPose = new Pose2d(0, 0, Math.toRadians(0));
            tapeFirstPoseRad = Math.toRadians(0);
            stackPickup = new Pose2d(0, 0, Math.toRadians(0));
            stackPickupRadians = Math.toRadians(0);
            tallPolePoseDrifted = new Pose2d(0, 0, Math.toRadians(0));
            tallPolePoseDriftedRad = Math.toRadians(0);
            tapeDrifted = new Pose2d(0,0,Math.toRadians(0));
            tapeDriftedRad = Math.toRadians(0);
            inputx = 0;
            inputy = 0;
            inputTheta = 0;
            stackPosRedef = new Pose2d(0, 0, 0);
        }

        telemetry.addData("Calculating trajectories...", "");
        telemetry.update();

        //Build first trajectory to the pole
        TrajectorySequence trajHomeToPole = mechdrive.trajectorySequenceBuilder(start)
                .addTemporalMarker(0.01,() -> {
                    Thread aprilscanner = new Thread( () -> {
                        //Scan for april tags
//                        couldFindTag = false;
                        ArrayList<AprilTagDetection> detections = apriltgsi.scan(1200);
                        if(detections.size() != 0) {
                            couldFindTag = true;
                            this.primaryDetection = detections.get(0);
                            apriltgsi.addDetectionToTelemetry(primaryDetection);
                        } else {
                            couldFindTag = false;
                            //figure out what to do
                        }
                        fourbar.closeClaw();
                    });
                    aprilscanner.start();
                })
                .addTemporalMarker(0.5, () -> fourbar.lift(1240, NullDoc.LIFT_TELEOP_SPEED))
                .addTemporalMarker(3, () -> fourbar.FBReachToIndex(1, 3))
                .setReversed(true)
                .splineToLinearHeading(tallPolePose, tallPolePoseRad)
                .build();

        //Build second trajectory to the stack
        TrajectorySequence trajPoleToTape = mechdrive.trajectorySequenceBuilder(trajHomeToPole.end())
                .setReversed(false)
                .addTemporalMarker(1, () -> {
                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.reach(0.72);
                })
                .splineToLinearHeading(tapeFirstPose,tapeFirstPoseRad)
                .build();
        //Build second trajectory to the stack
        TrajectorySequence trajTapeToStack = mechdrive.trajectorySequenceBuilder(trajPoleToTape.end())
                .setReversed(false)
                .splineToLinearHeading(stackPickup, stackPickupRadians) //to the wall
                .build();
        TrajectorySequence trajStackToPole = mechdrive.trajectorySequenceBuilder(trajTapeToStack.end())
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {
                    fourbar.FBReachToIndex(1, 3);
                })
                .splineToLinearHeading(tallPolePoseDrifted, tallPolePoseDriftedRad)
                .build();
        TrajectorySequence trajPoleToTape2 = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
                .setReversed(true)
                .addTemporalMarker(1, () -> {
                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.reach(0.75);
                })
                .splineToLinearHeading(tapeDrifted,tapeDriftedRad)
                .build();
        TrajectorySequence trajParkZn1Red = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(60,12,Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(57,10,Math.toRadians(0)), Math.toRadians(0))
                .turn(Math.toRadians(90))
                .forward(24)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn2Red = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .splineToLinearHeading(new Pose2d(36,12,Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(36,20,Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn3Red = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(12,12,Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(10,14,Math.toRadians(0)), Math.toRadians(-200))
                .turn(Math.toRadians(90))
                .forward(22)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();

        TrajectorySequence trajParkZn3Blue = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(60,12,Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-57,10,Math.toRadians(0)), Math.toRadians(0))
                .turn(Math.toRadians(90))
                .forward(16)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn2Blue = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .splineToLinearHeading(new Pose2d(36,12,Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-36,20,Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();
        TrajectorySequence trajParkZn1Blue = mechdrive.trajectorySequenceBuilder(trajStackToPole.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(12,12,Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-10,14,Math.toRadians(0)), Math.toRadians(-200))
                .turn(Math.toRadians(90))
                .forward(22)
                .addTemporalMarker(1, () -> {
                    fourbar.closeClaw();
//                    fourbar.FBReachToIndex(0, 1);
//                    fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
                })
                .build();


        telemetry.addData("Done calculating trajectories.", "");
        telemetry.addData("Alliance color", allianceColor.toString());
        telemetry.addData("Corner color", cornerColor.toString());
        telemetry.update();

        //initialize everything else
        fourbar.init(hardwareMap, telemetry);

        //grab cone
        fourbar.openClaw();
        fourbar.FBReachToIndex(1, 1);
        sleep(1000);
        fourbar.closeClaw();
        sleep(1000);
//        fourbar.FBReachToIndex(1,3);
//        tdp.warmUpCamera(telemetry);

        //open the camera
//        camsys.TopDown.setPipeline(tdp);

        //Wait for start.
        waitForStart();

        if(allianceColor == Label.RED){
            tdp.isTapeRed = true;
        } else if(allianceColor == Label.BLUE){
            tdp.isTapeRed = false;
        } else {
            telemetry.addData("Wtf happened", "");
            telemetry.update();
        }

        fourbar.FBReachToIndex(1,3);

        //Begin actual motion
        mechdrive.setPoseEstimate(start);

        //Go to pole and place cone
        mechdrive.followTrajectorySequence(trajHomeToPole);
        fourbar.liftWaitForStop();
        sleep(250);
        fourbar.FBReachToIndex(1, 2);
        sleep(500);
        fourbar.openClaw();

        //Go to stack and pick up cone
        mechdrive.followTrajectorySequence(trajPoleToTape);
        fourbar.lift(265, NullDoc.LIFT_TELEOP_DESC_SPEED);
        mechdrive.setPoseEstimate(tdp.calcPose(inputx,inputy,inputTheta, telemetry));
        mechdrive.followTrajectorySequence(trajTapeToStack);
        mechdrive.setPoseEstimate(stackPosRedef);
        fourbar.liftWaitForStop();
        fourbar.closeClaw();
        sleep(250);

        //Go back to the pole and place cone

        fourbar.lift(1240, NullDoc.LIFT_TELEOP_SPEED);
        mechdrive.followTrajectorySequence(trajStackToPole);
        sleep(500);
        fourbar.FBReachToIndex(1, 2);
        sleep(500);
        fourbar.openClaw();

        for (int i = 0; i < numStackCycles - 1; i++) {
            telemetry.addData("i", i);
            telemetry.update();
            //Go to stack and pick up cone
            mechdrive.followTrajectorySequence(trajPoleToTape2);
            fourbar.lift(195 /*200*/-(i*70), NullDoc.LIFT_TELEOP_DESC_SPEED);
//            fourbar.liftToPos(4-i, NullDoc.LIFT_TELEOP_SPEED );
            mechdrive.setPoseEstimate(tdp.calcPose(inputx, inputy, inputTheta, telemetry));
            mechdrive.followTrajectorySequence(trajTapeToStack);
            mechdrive.setPoseEstimate(stackPosRedef);
            fourbar.liftWaitForStop();
            sleep(150);
            fourbar.closeClaw();
            sleep(250);

            //Go back to the pole and place cone

            fourbar.lift(1240, NullDoc.LIFT_TELEOP_SPEED);
            mechdrive.followTrajectorySequence(trajStackToPole);
            //sleep(250);
            fourbar.FBReachToIndex(1, 2);
            sleep(500);
            fourbar.openClaw();
        }
//        mechdrive.followTrajectorySequence(trajPoleToTape2);
//        fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);

        //Park
        if(couldFindTag && cornerColor == Label.REDCORNER){
            switch (primaryDetection.id){
                case 0:
                    telemetry.addData("Parking Zone", 1);
                    telemetry.update();
                    mechdrive.followTrajectorySequence(trajParkZn1Red);
                    break;
                case 2:
                    telemetry.addData("Parking Zone", 2);
                    telemetry.update();
                    mechdrive.followTrajectorySequence(trajParkZn2Red);
                    break;
                case 1:
                    telemetry.addData("Parking Zone", 3);
                    telemetry.update();
                    mechdrive.followTrajectorySequence(trajParkZn3Red);
                    break;
            }
        } else if (couldFindTag && cornerColor == Label.BLUECORNER) {
            switch (primaryDetection.id){
                case 0:
                    telemetry.addData("Parking Zone", 1);
                    telemetry.update();
                    mechdrive.followTrajectorySequence(trajParkZn1Blue);
                    break;
                case 2:
                    telemetry.addData("Parking Zone", 2);
                    telemetry.update();
                    mechdrive.followTrajectorySequence(trajParkZn2Blue);
                    break;
                case 1:
                    telemetry.addData("Parking Zone", 3);
                    telemetry.update();
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

    public Label getAllianceColor(){
        return Label.NONE;
    }

}
