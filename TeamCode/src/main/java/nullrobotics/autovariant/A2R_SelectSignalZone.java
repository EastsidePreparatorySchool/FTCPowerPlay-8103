package nullrobotics.autovariant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.auto.norr.A2_SelectSignalZone;
import nullrobotics.lib.Label;

@Autonomous(name="[2R] Select Signal Zone", group="Z")
public class A2R_SelectSignalZone extends A2_SelectSignalZone {
    Label signalDirection = Label.RIGHT;

    @Override
    public Label getSignalDirection(){
        return signalDirection;
    }
}
