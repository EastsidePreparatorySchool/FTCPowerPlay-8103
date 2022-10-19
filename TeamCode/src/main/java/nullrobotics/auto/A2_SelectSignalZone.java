package nullrobotics.auto;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

import nullrobotics.lib.AprilTagImplementation;
import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;

@Autonomous(name="[A2] Select Signal Zone", group="Auto")
public class A2_SelectSignalZone extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();
    AprilTagImplementation camera = new AprilTagImplementation();

    @Override
    public void runOpMode() {
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);
        camera.init(hardwareMap, telemetry);

        fourbar.preloadCone();

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        chassis.drive(3);

        chassis.strafe(5);

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
                    chassis.strafe(-24);
                    chassis.drive(24);
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


}
