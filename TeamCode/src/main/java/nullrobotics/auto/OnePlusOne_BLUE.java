package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "One Plus Three BLUE CORNER",group = "Auto")
public class OnePlusOne_BLUE extends OnePlusOne {
    @Override
    public Label getCornerColor(){
        return Label.BLUECORNER;
    }
}
