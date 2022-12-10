package nullrobotics.lib;

public class Label {
    public Label(String asString){
        this.string = asString;
    }

    private String string;
    public String toString(){
        return this.string;
    }

    public static Label BLUE = new Label("Blue");
    public static Label RED = new Label("Red");
    public static Label LEFT = new Label("Left");
    public static Label RIGHT = new Label("Right");
    public static Label NONE = new Label("None");
    public static Label REDCORNER = new Label("RedCorner");
    public static Label BLUECORNER = new Label("BlueCorner");

}
