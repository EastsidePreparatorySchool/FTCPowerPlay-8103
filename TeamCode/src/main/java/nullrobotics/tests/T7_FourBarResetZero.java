package nullrobotics.tests;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;

@Autonomous(name="[T7] Four Bar Reset Zero", group="Test")
public class T7_FourBarResetZero extends LinearOpMode {

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

        fourbar.lift(700, 0.5);
        fourbar.liftWaitForStop();
        fourbar.reach(0.5);
        fourbar.tsleep(10000000);

    }

}
