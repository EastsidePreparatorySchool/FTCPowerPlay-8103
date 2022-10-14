package nullrobotics.tests;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;
import nullrobotics.lib.VoidLib;

@Autonomous(name="[T2] Angle 180 Deg", group="Test")
public class T2_Angle extends LinearOpMode {

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

        chassis.turn(VoidLib.DEFAULT_DRIVE_SPEED, 180);
    }


}
