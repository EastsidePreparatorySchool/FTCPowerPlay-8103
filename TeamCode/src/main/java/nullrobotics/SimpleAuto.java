package nullrobotics;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;
import nullrobotics.lib.VoidLib;

@Autonomous(name="Simple Auto", group="8103")
public class SimpleAuto extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();

    @Override
    public void runOpMode() {
        chassis.init(hardwareMap, telemetry);
        fourbar.init(hardwareMap, telemetry);

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        chassis.drive(VoidLib.DEFAULT_DRIVE_SPEED, (150));

        chassis.tsleep(5000);

        chassis.drive(VoidLib.DEFAULT_DRIVE_SPEED, (-30));

        chassis.tsleep(5000);

        chassis.turn(VoidLib.DEFAULT_DRIVE_SPEED, (90));

        chassis.tsleep(5000);

        chassis.drive(VoidLib.DEFAULT_DRIVE_SPEED, (-20));

        chassis.tsleep(1000);

        chassis.strafe(VoidLib.DEFAULT_DRIVE_SPEED, (-120));

        chassis.tsleep(1000);

        chassis.drive(VoidLib.DEFAULT_DRIVE_SPEED, (20));

        chassis.tsleep(1000);

        chassis.turn(VoidLib.DEFAULT_DRIVE_SPEED, (270));




    }

}
