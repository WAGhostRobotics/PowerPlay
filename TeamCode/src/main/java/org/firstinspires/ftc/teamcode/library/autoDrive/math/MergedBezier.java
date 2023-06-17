package org.firstinspires.ftc.teamcode.library.autoDrive.math;


public class MergedBezier extends Bezier{

    Bezier[] curves;
    private Point[] curvePoints;
    private Point[] curveDerivatives;
    private double[] curveHeadings;

    public Point[] getCurvePoints(){return curvePoints;}
    public Point[] getCurveDerivatives(){return curveDerivatives;}
    public double[] getCurveHeadings(){return curveHeadings;}

    public MergedBezier(Bezier... curves){
        this.curves = curves;

        generateCurve();
    }

    private void generateCurve(){

        curvePoints = new Point[(int)(1.0/tIncrement)];
        curveDerivatives = new Point[(int)(1.0/tIncrement)];
        curveHeadings = new double[(int)(1.0/tIncrement)];

        double currentT = 0;

        for(int i=0;i<curvePoints.length;i++){
            curvePoints[i] = getPoint(currentT);
            curveDerivatives[i] = getDerivative(currentT);
            curveHeadings[i] = getHeading(currentT);

            currentT += tIncrement;
        }
    }

    @Override
    public Point getPoint(double t){
        if(t==1) {
            return curves[curves.length-1].getPoint(1);
        }else {
            return curves[(int)(curves.length*t)].getPoint((curves.length*t)-((int)(curves.length*t)));
        }
    }

    @Override
    public double getHeading(double index){
        double t = index * tIncrement;
        return curves[(int)(curves.length*t)].getHeading(0);
    }

    @Override
    public Point getDerivative(double t){
        if(t==1) {
            return curves[curves.length-1].getDerivative(1);
        }else {
            return curves[(int)(curves.length*t)].getDerivative((curves.length*t)-((int)(curves.length*t)));
        }
    }

    @Override
    public double approximateLength() {
        double len = 0;
        for(Bezier i: curves){
            len += i.approximateLength();
        }

        return len;
    }

    @Override
    public Point getStartPoint(){
        return curves[0].getStartPoint();
    }


    @Override
    public Point getEndPoint() {
        return curves[curves.length-1].getEndPoint();
    }
}
