package nullrobotics.tests;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.lib.FourBarLift;

@Autonomous(name="[T5] Four Bar Test 2", group="Test")
public class T5_FourBarTest2 extends LinearOpMode {

    //declare opmode members
    FourBarLift fourbar = new FourBarLift();

    @Override
    public void runOpMode() {
        fourbar.init(hardwareMap, telemetry);

        //telemetry
        waitForStart();
        fourbar.LiftMotorL.setPower(0.1);
        fourbar.tsleep(1000);

    }

}
