package org.firstinspires.ftc.teamcode.library.autoDrive.math;


public class MergedBezier extends Bezier{

    Bezier[] curves;

    public MergedBezier(Bezier... curves) throws InterruptedException {
        this.curves = curves;

        for(int i=0;i<curves.length-1;i++){
            if(!curves[i].getEndPoint().equals(curves[i+1].getStartPoint())){
                throw new InterruptedException("Curves must be merged at the same point");
            }

//            if(!curves[i].getDerivative(1).equals(curves[i+1].getDerivative(0))){
//                throw new InterruptedException("Point of merging must be smooth");
//            }
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
    public double getHeading(double t){
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
