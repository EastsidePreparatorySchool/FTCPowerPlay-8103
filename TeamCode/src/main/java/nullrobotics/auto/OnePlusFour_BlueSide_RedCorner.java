package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "One Plus Four Blue Side Red Corner",group = "Auto")
public class OnePlusFour_BlueSide_RedCorner extends OnePlusFour {
    @Override
    public Label getCornerColor() {return Label.REDCORNER;};
}
