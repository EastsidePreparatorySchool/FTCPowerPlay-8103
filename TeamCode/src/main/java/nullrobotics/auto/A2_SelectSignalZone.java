package nullrobotics.auto;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        camera.onOpModeBegin();

        chassis.drive(3);

        chassis.turn(90);

        chassis.drive(-24);

        camera.scanForTags();

    }


}
