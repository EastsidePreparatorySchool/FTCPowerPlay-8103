package nullrobotics.lib;

public class Tuple {

    public double begDistance;
    public double endDistance;
    public double pdPercentRemaining;
    public double curveMultiplier;

    Tuple(double begDistance1, double endDistance1, double pdPercentRemaining1, double curveMultiplier1){
        this.begDistance = begDistance1;
        this.endDistance = endDistance1;
        this.pdPercentRemaining = pdPercentRemaining1;
        this.curveMultiplier = curveMultiplier1;
    }

}
