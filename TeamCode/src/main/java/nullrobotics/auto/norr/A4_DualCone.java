package nullrobotics.auto.norr;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.CameraSystem;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullDoc;
import nullrobotics.lib.NullHardware;

//@Autonomous(name="[4 Testing] Dual Cone", group="Auto")
public class A4_DualCone extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    CameraSystem camsys = new CameraSystem();
    AprilTagImplementation apriltgsi = new AprilTagImplementation();

    Label signalDirection = Label.NONE;

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

        chassis.drive(0.3, 8);

        fourbar.FBReachToIndex(0, 3); //raise cone so camera can see

        ArrayList<AprilTagDetection> detections = apriltgsi.scan(3500);

        //2nd. Drive towards the pole

        chassis.drive(47, true);

//        chassis.drive(-10);

        chassis.rest1();

        //3rd. Lift the slides and strafe in

        fourbar.lift(1200, NullDoc.LIFT_TELEOP_SPEED);
        fourbar.liftWaitForStop();

        chassis.rest1();

        if(signalDirection == Label.REDCORNER){
            chassis.strafe(0.3, 11.5);
        } else if (signalDirection == Label.BLUECORNER){
            chassis.strafe(0.3, -13);
        } else {
            telemetry.addData("Unknown label", signalDirection.toString());
            telemetry.update();
        }

        //4th. Drive in, lower the fourbar for accuracy (Claire's Methodd) and open the claw

        this.place(3.5);

        this.cycle(250);

        this.place(3.5);

//        this.cycle(250);
//
//        this.place();

        // (the rest of the program)
        fourbar.lift(0, NullDoc.LIFT_TELEOP_DESC_SPEED);
        fourbar.liftWaitForStop();

        //6th. Go to the right zone

        switch (detections.get(0).id){
            case 0:
                //Zone 1
//                chassis.strafe(-24);
                if(signalDirection == Label.REDCORNER){
                    chassis.strafe(-36, true);
                } else if (signalDirection == Label.BLUECORNER){
                    chassis.strafe(-12, true);
                }
                break;
            case 2:
                if(signalDirection == Label.REDCORNER){
                    chassis.strafe(-12, true);
                } else if (signalDirection == Label.BLUECORNER){
                    chassis.strafe(12, true);
                }
                break;
            case 1:
                //Zone 3
//                chassis.strafe(24);
                if(signalDirection == Label.REDCORNER){
                    chassis.strafe(12, true);
                } else if (signalDirection == Label.BLUECORNER){
                    chassis.strafe(36, true);
                }
                break;
        }

        chassis.drive(-5);

    }

    public Label getSignalDirection(){
        return Label.NONE;
    }

    public void cycle(int htTicks){
        if(this.signalDirection == Label.BLUECORNER) {
            chassis.turn(-91);
        } else if(this.signalDirection == Label.REDCORNER){
            chassis.turn(91);
        }

        fourbar.FBReachToIndex(0, 1);
        fourbar.openClaw();

        fourbar.lift(htTicks, NullDoc.LIFT_TELEOP_SPEED);
        fourbar.liftWaitForStop();

        chassis.rest1();

        chassis.drive(45);

        chassis.rest1();

        fourbar.closeClaw();

        chassis.rest1();

        fourbar.lift(htTicks+200, NullDoc.LIFT_TELEOP_SPEED);
        fourbar.liftWaitForStop();

        chassis.rest1();

        chassis.drive(-45);
        if(this.signalDirection == Label.BLUECORNER){
            chassis.strafe(0.3, -2);
        } else if (this.signalDirection == Label.REDCORNER) {
            chassis.strafe(0.3, -2);
        }

        chassis.rest1();

        fourbar.FBReachToIndex(0, 3);

        fourbar.lift(1250, NullDoc.LIFT_TELEOP_SPEED);
        fourbar.liftWaitForStop();

        if(this.signalDirection == Label.BLUECORNER) {
            chassis.turn(90);
        } else if(this.signalDirection == Label.REDCORNER){
            chassis.turn(-90);
        }
    }

    public void place(double fwd){
        chassis.rest1();

        chassis.drive(0.4, fwd);

        chassis.rest1();

        fourbar.FBReachToIndex(0, 2);

        chassis.rest1();

        fourbar.openClaw();

        chassis.rest1();

        //5th. Back out and lower the slides

        chassis.drive(0.4, -fwd);

        chassis.rest1();
    }

}
