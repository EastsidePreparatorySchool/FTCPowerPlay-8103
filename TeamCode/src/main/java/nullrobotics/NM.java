package nullrobotics;

public class NM {
    //    Ticks per centimeter

    private static final double TicksPerCm = 17;
    public static int cm(int num){
        return (int) (num * TicksPerCm);
    }

    private static final double TicksPerDeg = 8;
    public static int deg(int num){
        return (int) (num * TicksPerDeg);
    }

    //90º = 750 ticks
//    65cm = 1000 ticks -> 15 ticks per cm

//    Default speeds for things
    public static final double DEFAULTDRIVESPEED = 0.65;


}
