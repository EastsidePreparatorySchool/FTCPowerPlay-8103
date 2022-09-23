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

        robot.FourBarLift.rise(20, 0.05);

        robot.FourBarLift.reach(45);
        robot.tsleep(5000);
        robot.FourBarLift.reach(-45);

    }

}
