package nullrobotics;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Four Bar Test", group="8103")
public class FourBarTest extends LinearOpMode {

    //declare opmode members
    NullHardware robot = new NullHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

            robot.FourBarLift.telServoPositions();

//            robot.FourBarLift.riseBy(15, 0.3);
//
//            robot.FourBarLift.reach(0.8);
//
//            robot.tsleep(3000);
//
//            robot.FourBarLift.reach(1);
//
//            robot.tsleep(3000);

            robot.FourBarLift.open();
            robot.tsleep(3000);
            robot.FourBarLift.close();
            robot.tsleep(1000);

            robot.FourBarLift.reach(0.9);

            robot.tsleep(1000);

            robot.FourBarLift.riseBy(45, 0.3);

            robot.tsleep(1000);

//            robot.FourBarLift.reach(1);
//
//            robot.tsleep(1000);

            robot.FourBarLift.open();

            robot.tsleep(1000);

            robot.FourBarLift.reach(0.8);

            robot.FourBarLift.riseBy(-10, 0.2);

//            robot.FourBarLift.endLiftMovement();

            robot.tsleep(10000);
    }

}
