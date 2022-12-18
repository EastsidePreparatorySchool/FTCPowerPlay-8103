package nullrobotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import nullrobotics.lib.Label;

@Autonomous(name = "One Plus Three RED CORNER",group = "Z")
public class OnePlusOne_RED extends OnePlusOne {
    @Override
    public Label getCornerColor(){
        return Label.REDCORNER;
    }
}
