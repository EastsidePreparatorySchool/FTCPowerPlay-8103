package nullrobotics.tests;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.lib.FourBarLift;
import nullrobotics.lib.NullHardware;

@Autonomous(name="[T6] Four Bar Test", group="Test")
public class T6_FourBarTest extends LinearOpMode {

    //declare opmode members
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

            fourbar.telPositions();

//            fourbar.riseBy(15, 0.3);
//
//            fourbar.reach(0.8);
//
//            chassis.tsleep(3000);
//
//            fourbar.reach(1);
//
//            chassis.tsleep(3000);

            fourbar.openClaw();
            chassis.tsleep(3000);
            fourbar.closeClaw();
            chassis.tsleep(1000);

            fourbar.reach(0.9);

            chassis.tsleep(1000);

            fourbar.lift(45, 0.3);

            chassis.tsleep(1000);

//            fourbar.reach(1);
//
//            chassis.tsleep(1000);

            fourbar.openClaw();

            chassis.tsleep(1000);

            fourbar.reach(0.8);

            fourbar.lift(-45, 0.2);

//            fourbar.endLiftMovement();

            chassis.tsleep(10000);
    }

}
