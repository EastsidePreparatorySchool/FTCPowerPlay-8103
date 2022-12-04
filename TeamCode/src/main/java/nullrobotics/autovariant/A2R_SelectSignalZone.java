package nullrobotics.autovariant;

import nullrobotics.auto.norr.A2_SelectSignalZone;
import nullrobotics.lib.Label;

//@Autonomous(name="[2R] Select Signal Zone", group="Auto")
public class A2R_SelectSignalZone extends A2_SelectSignalZone {
    Label signalDirection = Label.RIGHT;

    @Override
    public Label getSignalDirection(){
        return signalDirection;
    }
}
