package nullrobotics.auto;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullHardware;
import nullrobotics.lib.VoidLib;

@Autonomous(name="[3] Cone Cycle", group="Auto")
public class A3_ConeCycle extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    AprilTagImplementation camera = new AprilTagImplementation();

    Label signalDirection = Label.RIGHT;

    @Override
    public void runOpMode() {
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        camera.init(hardwareMap, telemetry);

//        signalDirection = this.getSignalDirection();

        fourbar.preloadCone();

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        //1st. Detect signal cone
        chassis.drive(4);

        chassis.turn(90);

        if(signalDirection == Label.LEFT){
            chassis.drive(12);
        } else if (signalDirection == Label.RIGHT){
            chassis.drive(-12);
        } else {
            telemetry.addData("Unknown Label", signalDirection.toString());
            telemetry.update();
            stop();
        }

        chassis.turn(-90);

        chassis.drive(4);

        fourbar.FBReachToIndex(0, 1); //raise cone so camera can see

        ArrayList<AprilTagDetection> detections = camera.scan();

        //2nd. Score preload cone
        fourbar.FBReachToIndex(0, 0);

        //Move back and lower fourbar
        chassis.drive(-5);

        //Go around Zone 2 to move into position
//        chassis.strafe(-25);
        chassis.turn(90);
        chassis.drive(24);
        chassis.turn(-90);

        chassis.drive(0.2, -10); //square back against the wall
        chassis.drive(52);

        chassis.strafe(13);
//        chassis.turn(90);
//        chassis.drive(-13);
//        chassis.turn(-90);

        //Lift
        fourbar.lift(1300, VoidLib.LIFT_TELEOP_SPEED);
        fourbar.liftWaitForStop();
        chassis.tsleep(2000);
        fourbar.FBReachToIndex(0, 1);

        chassis.drive(3);

        fourbar.lift(1200, 0.1);
        fourbar.liftWaitForStop();
        fourbar.openClaw();

        chassis.drive(-4);
        fourbar.lift(0,0.4);

        //3rd. Park


    }

    public Label getSignalDirection(){
        return Label.NONE;
    }


}
