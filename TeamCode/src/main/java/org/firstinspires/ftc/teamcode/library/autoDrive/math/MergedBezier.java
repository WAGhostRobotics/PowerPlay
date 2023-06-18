package org.firstinspires.ftc.teamcode.library.autoDrive.math;


public class MergedBezier extends Bezier{

    Bezier[] curves;

    double[] mergePoints;

    public MergedBezier(Bezier... curves){
        this.curves = curves;

        mergePoints = new double[curves.length];
        double len = approximateLength();
        for(int i=0;i< mergePoints.length-1;i++){
            mergePoints[i] = curves[i].approximateLength()/len + ((i==0) ? 0 : mergePoints[i-1]);
        }

        mergePoints[mergePoints.length-1] = 1;

        generateCurve();
    }

    @Override
    public Point getPoint(double t){



        if(t>=1) {
            return curves[curves.length-1].getPoint(1);
        }else {
            int i=0;
            while (t > mergePoints[i]){i++;}
            return curves[i].getPoint((i==0) ? (t/mergePoints[i]):((t-mergePoints[i-1])/(mergePoints[i]-mergePoints[i-1])));
        }
    }

    @Override
    public double getHeading(double t){



        if(t>=1) {
            return curves[curves.length-1].getHeading(0);
        }else {
            int i=0;
            while (t > mergePoints[i]){i++;}
            return curves[i].getHeading((i==0) ? (t/mergePoints[i]):((t-mergePoints[i-1])/(mergePoints[i]-mergePoints[i-1])));
        }
    }

    @Override
    public Point getDerivative(double t){


        if(t>=1) {
            return curves[curves.length-1].getDerivative(1);
        }else {
            int i=0;
            while (t > mergePoints[i]){i++;}
            return curves[i].getDerivative((i==0) ? (t/mergePoints[i]):((t-mergePoints[i-1])/(mergePoints[i]-mergePoints[i-1])));
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
