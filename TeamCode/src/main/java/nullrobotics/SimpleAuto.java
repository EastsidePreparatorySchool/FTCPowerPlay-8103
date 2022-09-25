package nullrobotics;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Simple Auto", group="8103")
public class SimpleAuto extends LinearOpMode {

    //declare opmode members
    NullHardware robot = new NullHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

        robot.drive(VoidLib.DEFAULTDRIVESPEED, (150));

        robot.tsleep(5000);

        robot.drive(VoidLib.DEFAULTDRIVESPEED, (-30));

        robot.tsleep(5000);

        robot.turn(VoidLib.DEFAULTDRIVESPEED, (90));

        robot.tsleep(5000);

        robot.drive(VoidLib.DEFAULTDRIVESPEED, (-20));

        robot.tsleep(1000);

        robot.strafe(VoidLib.DEFAULTDRIVESPEED, (-120));

        robot.tsleep(1000);

        robot.drive(VoidLib.DEFAULTDRIVESPEED, (20));

        robot.tsleep(1000);

        robot.turn(VoidLib.DEFAULTDRIVESPEED, (270));




    }

}
