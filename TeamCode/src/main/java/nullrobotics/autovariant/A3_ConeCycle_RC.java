package nullrobotics.autovariant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.auto.A3_ConeCycle;
import nullrobotics.lib.Label;

@Autonomous(name = "[3 RC] Cone Cycle (Red Corner)", group = "Auto")
public class A3_ConeCycle_RC extends A3_ConeCycle {
    @Override
    public Label getSignalDirection(){
        return Label.REDCORNER;
    }
}
