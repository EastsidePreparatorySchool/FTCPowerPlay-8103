package nullrobotics.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.CameraSystem;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullHardware;

//Detect tag on the signal sleeve, then go to the right zone.
//Is the same for all four quarters of the field.

//Setup position: right in front of short yellow pole
//Choose L or R by: is the Signal Cone to the LEFT or RIGHT of the robot's starting position?

@Autonomous(name="[2] Select Signal Zone", group="Auto")
public class A2_SelectSignalZone extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    CameraSystem camsys = new CameraSystem();
    AprilTagImplementation apriltgsi = new AprilTagImplementation();
    Label signalDirection;

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

        fourbar.FBReachToIndex(0, 3); //raise cone so camera can see

        chassis.drive(0.5, 3);

        chassis.drive(5);

        ArrayList<AprilTagDetection> detections = apriltgsi.scan();

        if(detections == null){
            //If it can't find a tag, then park normally.
            chassis.drive(-5);
            chassis.strafe_classic(-5);
            chassis.turn(90);
            chassis.drive(-24);

        } else {

            AprilTagDetection primaryDetection = detections.get(0);

            //Convey back primary detection
            apriltgsi.addDetectionToTelemetry(primaryDetection);
            telemetry.update();

//            chassis.tsleep(5000);

            chassis.drive(-6);

            fourbar.FBReachToIndex(0, 1);

            switch (primaryDetection.id) {
                case 0:
                    //Zone 1
                    chassis.strafe_classic(-26);
                    chassis.drive(30);
                    break;
                case 2:
                    /*
                    //Zone 2 (go around to not knock over cone)
                    chassis.strafe_classic(-24);
                    chassis.drive(52);
                    chassis.strafe_classic(29);
                    break;
                    */
                    //Zone 2 (just smash through)
                    chassis.drive(52);
                    break;
                case 1:
                    //Zone 3
                    chassis.strafe_classic(31);
                    chassis.drive(28);
                    break;
            }

        }

    }

    public Label getSignalDirection(){
        return Label.NONE;
    }

}
