package org.firstinspires.ftc.teamcode.library.math;

public class Bezier {

    Point[] waypoints;

    double pow;
    double pow2;
    double pow3;
    double pow4;


    public Bezier(Point... waypoints) {
        this.waypoints = waypoints;
    }

    public Point getPoint(double t){

        double x = 0;

        double y = 0;

        int n = waypoints.length-1;

        for (int i = 0; i <= n; i++) {

            double b = choose(n, i) * Math.pow((1-t), n-i) * Math.pow(t, i);
            x +=  b * waypoints[i].getX();

            y += b * waypoints[i].getY();

        }

        return new Point(x, y);

    }

    public Point getDerivative(double t){

        double x = 0;

        double y = 0;

        int n = waypoints.length-1;

        for (int i = 0; i <= n-1; i++) {

            double b = choose(n-1, i) * Math.pow((1-t), n-i-1) * Math.pow(t, i);

            x += b * (waypoints[i+1].getX()-waypoints[i].getX());
            y += b * (waypoints[i+1].getY()-waypoints[i].getY());
        }

        x *= n;
        y *= n;

        return new Point(x, y);

    }

    public Point getDerivative2(double t){

        double x = 0;

        double y = 0;

        int n = waypoints.length-1;

        for (int i = 0; i <= n; i++) {

            double b = choose(n, i);

            if(n-i==0){
                pow = Math.pow(t, i - 1);
                x += b * i * pow * waypoints[i].getX();
                y += b * i * pow * waypoints[i].getY();
            }else if(i==0){
                pow = Math.pow(1 - t, n - i - 1);
                x += -1 * b * (n-i) * pow * waypoints[i].getX();
                y += -1 * b * (n-i) * pow * waypoints[i].getY();
            }else{
                pow = Math.pow(1-t, n-i-1);
                pow2 = Math.pow(t, i);
                pow3 = Math.pow(1-t, n-i);
                pow4 = Math.pow(t, i-1);

                x += -1 * b * (n-i) * pow * pow2 * waypoints[i].getX() + b * pow3 * i * pow4 * waypoints[i].getX();
                y += -1 * b * (n-i) * pow * pow2 * waypoints[i].getY() + b * pow3 * i * pow4 * waypoints[i].getY();
            }


        }

        return new Point(x, y);

    }


    public int choose(int n, int k) {
        return (factorial(n))/((factorial(k))*factorial(n-k));
    }

    public int factorial(int n) {
        if (n == 0)
            return 1;
        else 
            return(n * factorial(n-1));
    }


    public double approximateLength() {
        double distance = 0;
        for (double i=0.05; i<=1; i+=0.05) {
            Point pt = getPoint(i);
            Point pt2 = getPoint(i-0.05);
            distance += Math.hypot(pt.getX()-pt2.getX(), pt.getY()-pt2.getY());
        }

        return distance;
    }

    public Point getEndPoint() {
        return waypoints[waypoints.length-1];
    }

}
