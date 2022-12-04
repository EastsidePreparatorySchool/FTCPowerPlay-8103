package nullrobotics.autovariant;

import nullrobotics.auto.norr.A2_SelectSignalZone;
import nullrobotics.lib.Label;

//@Autonomous(name="[2L] Select Signal Zone", group="Auto")
public class A2L_SelectSignalZone extends A2_SelectSignalZone {
    Label signalDirection = Label.LEFT;

    @Override
    public Label getSignalDirection(){
        return signalDirection;
    }
}
