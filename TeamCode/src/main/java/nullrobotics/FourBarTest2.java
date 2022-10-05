package nullrobotics;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;

@Autonomous(name="Four Bar Test 2", group="8103")
public class FourBarTest2 extends LinearOpMode {

    //declare opmode members
    FourBarLift fourbar = new FourBarLift();

    @Override
    public void runOpMode() {
        fourbar.init(hardwareMap, telemetry);

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        fourbar.debug_SetLiftMotorPwr(0.6);

        fourbar.tsleep(2000);


    }

}
