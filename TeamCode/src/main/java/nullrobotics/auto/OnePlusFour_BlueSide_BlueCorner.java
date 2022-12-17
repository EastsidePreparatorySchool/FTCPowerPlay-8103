package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "One Plus Four Blue Side Blue Corner",group = "Auto")
public class OnePlusFour_BlueSide_BlueCorner extends OnePlusFour {
    @Override
    public Label getCornerColor() {return Label.BLUECORNER;};
}
