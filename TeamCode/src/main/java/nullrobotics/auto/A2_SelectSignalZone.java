package nullrobotics.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.Label;
import nullrobotics.lib.NullHardware;

//Detect tag on the signal sleeve, then go to the right zone.
//Is the same for all four quarters of the field.

//Setup position: right in front of short yellow pole
//Choose L or R by: is the Signal Cone to the LEFT or RIGHT of the robot's starting position?

public class A2_SelectSignalZone extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    AprilTagImplementation camera = new AprilTagImplementation();
    Label signalDirection;

    @Override
    public void runOpMode() {
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        camera.init(hardwareMap, telemetry);

        signalDirection = this.getSignalDirection();

        fourbar.preloadCone();

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        fourbar.FBReachToIndex(0, 1); //raise cone so camera can see

        chassis.drive(0.5, 3);

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

        chassis.drive(5);

        ArrayList<AprilTagDetection> detections = camera.scan();

        if(detections == null){
            //If it can't find a tag, then park normally.
            chassis.drive(-5);
            chassis.strafe(-5);
            chassis.turn(90);
            chassis.drive(-24);

        } else {

            AprilTagDetection primaryDetection = detections.get(0);

            //Convey back primary detection
            camera.addDetectionToTelemetry(primaryDetection);
            telemetry.update();

//            chassis.tsleep(5000);

            chassis.drive(-6);

            fourbar.FBReachToIndex(0, 0);

            switch (primaryDetection.id) {
                case 0:
                    //Zone 1
                    chassis.strafe(-26);
                    chassis.drive(30);
                    break;
                case 2:
                    //Zone 2 (go around to not knock over cone)
                    chassis.strafe(-24);
                    chassis.drive(52);
                    chassis.strafe(29);
                    break;
                case 1:
                    //Zone 3
                    chassis.strafe(31);
                    chassis.drive(28);
                    break;
            }

        }

    }

    public Label getSignalDirection(){
        return Label.NONE;
    }

}
