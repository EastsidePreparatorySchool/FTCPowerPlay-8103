package nullrobotics;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import nullrobotics.lib.NullHardware;

@Autonomous(name="Direction Test A", group="8103")
public class DirectionTestA extends LinearOpMode {

    //declare OpMode members
    NullHardware robot = new NullHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        robot.drive(0.5, 100);

        robot.tsleep(1000);

        robot.drive(0.5, -100);

        robot.tsleep(1000);

        robot.strafe(0.5, 100);

        robot.tsleep(1000);

        robot.strafe(0.5, -100);

        robot.tsleep(1000);

        robot.turn(0.5, 180);

        robot.tsleep(1000);

        robot.turn(.5, -180);

    }

}
