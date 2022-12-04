package nullrobotics.autovariant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.auto.norr.A3_SingleCone;
import nullrobotics.lib.Label;

@Autonomous(name = "[3] Single Cone (Blue Corner)", group = "Auto")
public class A3_SingleCone_BC extends A3_SingleCone {
    @Override
    public Label getSignalDirection(){
        return Label.BLUECORNER;
    }
}
