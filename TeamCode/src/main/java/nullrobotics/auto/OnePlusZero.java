package nullrobotics.auto;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.CameraSystem;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullHardware;
import nullrobotics.lib.VoidLib;

//@Autonomous(name="[3T] Cone Cycle", group="Auto")
public class OnePlusZero extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    CameraSystem camsys = new CameraSystem();
    AprilTagImplementation apriltgsi = new AprilTagImplementation();

    Label signalDirection = Label.NONE;
//    Label signalDirection = Label.BLUECORNER;

    @Override
    public void runOpMode() {
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        camsys.init(hardwareMap);
        apriltgsi.init(hardwareMap, telemetry, camsys.Front);

        signalDirection = this.getSignalDirection();

        fourbar.preloadCone();

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        //1st. Detect signal cone

        chassis.drive(0.3, -8);

        fourbar.FBReachToIndex(0, 3); //raise cone so camera can see

        ArrayList<AprilTagDetection> detections = apriltgsi.scan(3500);

        //2nd. Drive towards the pole

        chassis.drive(-48, true);

        chassis.tsleep(500);

        //3rd. Lift the slides and strafe in

        fourbar.lift(1200, VoidLib.LIFT_TELEOP_SPEED);
        fourbar.liftWaitForStop();

        chassis.tsleep(1000);

        double strafeAmt = 12;
        if(signalDirection == Label.REDCORNER){
            chassis.strafe(0.3, strafeAmt);
        } else if (signalDirection == Label.BLUECORNER){
            chassis.strafe(0.3, -strafeAmt);
        }

        //4th. Drive in, lower the fourbar for accuracy (Claire's Methodd) and open the claw

        chassis.tsleep(1000);

        chassis.drive(0.4, -3);

        chassis.tsleep(1000);

        fourbar.FBReachToIndex(0, 2);

        chassis.tsleep(1000);

        fourbar.openClaw();

        chassis.tsleep(1000);

        //5th. Back out and lower the slides

        chassis.drive(0.4, 3);

        fourbar.tsleep(1000);

        if(signalDirection == Label.REDCORNER){
            chassis.strafe(-12, true);
        } else if (signalDirection == Label.BLUECORNER){
            chassis.strafe(12, true);
        }
        fourbar.lift(0, VoidLib.LIFT_TELEOP_DESC_SPEED);
        fourbar.liftWaitForStop();

        //6th. Go to the right zone

        switch (detections.get(0).id){
            case 0:
                //Zone 1
                chassis.strafe(-24);
                break;
            case 2:
                break;
            case 1:
                //Zone 3
                chassis.strafe(24);
                break;
        }

        chassis.drive(5);

    }

    public Label getSignalDirection(){
        return Label.NONE;
    }


}
