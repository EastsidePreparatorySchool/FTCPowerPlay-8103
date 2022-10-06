package nullrobotics;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;
import nullrobotics.lib.VoidLib;

@Autonomous(name="Four Bar Reset Zero", group="8103")
public class FourBarResetZero extends LinearOpMode {

    //Declare OpMode members
    NullHardware chassis = new NullHardware();
    FourBarLift fourbar = new FourBarLift();

    @Override
    public void runOpMode() {
        fourbar.init(hardwareMap, telemetry);

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        fourbar.reach(0.5);
        fourbar.tsleep(10000000);

    }

}
