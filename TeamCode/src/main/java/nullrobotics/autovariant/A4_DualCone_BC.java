package nullrobotics.autovariant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.auto.A4_DualCone;
import nullrobotics.lib.Label;

@Autonomous(name = "[4] Dual Cone (Blue Corner)", group = "Auto")
public class A4_DualCone_BC extends A4_DualCone {
    @Override
    public Label getSignalDirection(){
        return Label.BLUECORNER;
    }
}