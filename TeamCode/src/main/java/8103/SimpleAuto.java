package dev.benor;

//Terminator, Destroyer of All, Bane of Android Studio

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Simple Auto", group="8103")
public class SimpleAuto extends LinearOpMode {

    //declare opmode members
    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        //telemetry
        telemetry.addData("Status", "started");
        telemetry.update();

        waitForStart();

//        robot.drive(Owl.DEFAULTDRIVESPEED, 1000);
//        robot.strafe(0.3, 50* Val.CM);
//        robot.turn(0.3, Val.deg(90));

        robot.drive(Owl.DEFAULTDRIVESPEED, Owl.cm(150));

        robot.tsleep(5000);

        robot.drive(Owl.DEFAULTDRIVESPEED, Owl.cm(-30));

        robot.tsleep(5000);

        robot.turn(Owl.DEFAULTDRIVESPEED, Owl.deg(90));

        robot.tsleep(5000);

        robot.drive(Owl.DEFAULTDRIVESPEED, Owl.cm(-20));

        robot.tsleep(1000);

        robot.strafe(Owl.DEFAULTDRIVESPEED, Owl.cm(-120));

        robot.tsleep(1000);

        robot.drive(Owl.DEFAULTDRIVESPEED, Owl.cm(20));

        robot.tsleep(1000);

        robot.turn(Owl.DEFAULTDRIVESPEED, Owl.deg(270));




    }

}
