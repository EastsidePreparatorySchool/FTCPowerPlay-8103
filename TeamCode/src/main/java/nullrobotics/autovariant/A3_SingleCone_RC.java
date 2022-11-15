package nullrobotics.autovariant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.auto.A3_SingleCone;
import nullrobotics.lib.Label;

@Autonomous(name = "[3] Single Cone (Red Corner)", group = "Auto")
public class A3_SingleCone_RC extends A3_SingleCone {
    @Override
    public Label getSignalDirection(){
        return Label.REDCORNER;
    }
}