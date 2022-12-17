package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "One Plus Four Red Side Red Corner",group = "Auto")
public class OnePlusFour_RedSide_RedCorner extends OnePlusFour {
    @Override
    public Label getCornerColor() {return Label.REDCORNER;};
}
