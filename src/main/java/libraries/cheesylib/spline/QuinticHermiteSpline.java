package libraries.cheesylib.spline;

import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.Util;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public class QuinticHermiteSpline extends Spline {
    private static final double kEpsilon = 1e-5;
    private static final double kStepSize = 1.0;
    private static final double kMinDelta = 0.001;
    private static final int kSamples = 100;
    private static final int kMaxIterations = 100;

    public double x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1;
    public double theta0, theta1, dtheta0, dtheta1, ddtheta0, ddtheta1;
    private double ax, bx, cx, dx, ex, fx, ay, by, cy, dy, ey, fy;
    private double atheta, btheta, ctheta, dtheta, etheta, ftheta;

    /**
     * @param p0 The starting pose of the spline
     * @param p1 The ending pose of the spline
     */
    public QuinticHermiteSpline(Pose2d p0, Pose2d p1) {
        double scale = 1.2 * p0.getTranslation().distance(p1.getTranslation());
        x0 = p0.getTranslation().x();
        x1 = p1.getTranslation().x();
        dx0 = p0.getRotation().cos() * scale;
        dx1 = p1.getRotation().cos() * scale;
        ddx0 = 0;
        ddx1 = 0;
        y0 = p0.getTranslation().y();
        y1 = p1.getTranslation().y();
        dy0 = p0.getRotation().sin() * scale;
        dy1 = p1.getRotation().sin() * scale;
        ddy0 = 0;
        ddy1 = 0;

        //Orientation
        var angle = Util.normalize_angle_positive(Math.atan2(y1 - y0, x1 - x0));
        theta0 = angle;
        theta1 = angle;
        dtheta0 = 0.0;
        dtheta1 = 0.0;
        ddtheta0 = 0.0;
        ddtheta1 = 0.0;

        computeCoefficients();
    }

    public QuinticHermiteSpline(Pose2d p0, Pose2d p1, double theta0, double theta1) {
        this(p0, p1);
        this.theta0 = theta0;
        this.theta1 = theta1;

        computeCoefficients();
    }

    /**
     * Used by the curvature optimization function
     */
    public QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1,
                                double y0, double y1, double dy0, double dy1, double ddy0, double ddy1) {
        this.x0 = x0;
        this.x1 = x1;
        this.dx0 = dx0;
        this.dx1 = dx1;
        this.ddx0 = ddx0;
        this.ddx1 = ddx1;

        this.y0 = y0;
        this.y1 = y1;
        this.dy0 = dy0;
        this.dy1 = dy1;
        this.ddy0 = ddy0;
        this.ddy1 = ddy1;

        computeCoefficients();
    }

    public QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1,
                                double y0, double y1, double dy0, double dy1, double ddy0, double ddy1,
                                double theta0, double theta1, double dtheta0, double dtheta1, double ddtheta0, double ddtheta1) {
        this.x0 = x0;
        this.x1 = x1;
        this.dx0 = dx0;
        this.dx1 = dx1;
        this.ddx0 = ddx0;
        this.ddx1 = ddx1;

        this.y0 = y0;
        this.y1 = y1;
        this.dy0 = dy0;
        this.dy1 = dy1;
        this.ddy0 = ddy0;
        this.ddy1 = ddy1;

        this.theta0 = theta0;
        this.theta1 = theta1;
        this.dtheta0 = dtheta0;
        this.dtheta1 = dtheta1;
        this.ddtheta0 = ddtheta0;
        this.ddtheta1 = ddtheta1;

        computeCoefficients();
    }


    /**
     * Re-arranges the spline into an at^5 + bt^4 + ... + f form for simpler computations
     */
    public void computeCoefficients() {
        ax = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
        bx = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
        cx = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
        dx = 0.5 * ddx0;
        ex = dx0;
        fx = x0;

        ay = -6 * y0 - 3 * dy0 - 0.5 * ddy0 + 0.5 * ddy1 - 3 * dy1 + 6 * y1;
        by = 15 * y0 + 8 * dy0 + 1.5 * ddy0 - ddy1 + 7 * dy1 - 15 * y1;
        cy = -10 * y0 - 6 * dy0 - 1.5 * ddy0 + 0.5 * ddy1 - 4 * dy1 + 10 * y1;
        dy = 0.5 * ddy0;
        ey = dy0;
        fy = y0;

        atheta = -6 * theta0 - 3 * dtheta0 - 0.5 * ddtheta0 + 0.5 * ddtheta1 - 3 * dtheta1 + 6 * theta1;
        btheta = 15 * theta0 + 8 * dtheta0 + 1.5 * ddtheta0 - ddy1 + 7 * dtheta1 - 15 * theta1;
        ctheta = -10 * theta0 - 6 * dtheta0 - 1.5 * ddtheta0 + 0.5 * ddtheta1 - 4 * dtheta1 + 10 * theta1;
        dtheta = 0.5 * ddtheta0;
        etheta = dtheta0;
        ftheta = theta0;

    }

    public Pose2d getStartPose() {
        return new Pose2d(
                new Translation2d(x0, y0),
                new Rotation2d(dx0, dy0, true)
        );
    }

    public Pose2d getEndPose() {
        return new Pose2d(
                new Translation2d(x1, y1),
                new Rotation2d(dx1, dy1, true)
        );
    }

    /**
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    @Override
    public Translation2d getPoint(double t) {
        double x = ax * t * t * t * t * t + bx * t * t * t * t + cx * t * t * t + dx * t * t + ex * t + fx;
        double y = ay * t * t * t * t * t + by * t * t * t * t + cy * t * t * t + dy * t * t + ey * t + fy;
        return new Translation2d(x, y);
    }

    private double dx(double t) {
        return 5 * ax * t * t * t * t + 4 * bx * t * t * t + 3 * cx * t * t + 2 * dx * t + ex;
    }

    private double dy(double t) {
        return 5 * ay * t * t * t * t + 4 * by * t * t * t + 3 * cy * t * t + 2 * dy * t + ey;
    }

    private double ddx(double t) {
        return 20 * ax * t * t * t + 12 * bx * t * t + 6 * cx * t + 2 * dx;
    }

    private double ddy(double t) {
        return 20 * ay * t * t * t + 12 * by * t * t + 6 * cy * t + 2 * dy;
    }

    private double dddx(double t) {
        return 60 * ax * t * t + 24 * bx * t + 6 * cx;
    }

    private double dddy(double t) {
        return 60 * ay * t * t + 24 * by * t + 6 * cy;
    }

    @Override
    public double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }

    @Override
    public double getCurvature(double t) {
        return (dx(t) * ddy(t) - ddx(t) * dy(t)) / ((dx(t) * dx(t) + dy(t) * dy(t)) * Math.sqrt((dx(t) * dx(t) + dy
                (t) * dy(t))));
    }

    @Override
    public double getDCurvature(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        if (dx2dy2 == 0.0) {
            return 0.0;
        }

        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }

    private double dCurvature2(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        if (dx2dy2 == 0.0) {
            return 0.0;
        }

        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
    }

    @Override
    public Rotation2d getHeading(double t) {
        return new Rotation2d(dx(t), dy(t), true);
    }

    /**
     * @return integral of dCurvature^2 over the length of the spline
     */
    private double sumDCurvature2() {
        double dt = 1.0 / kSamples;
        double sum = 0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * dCurvature2(t));
        }
        return sum;
    }

    /**
     * @return integral of dCurvature^2 over the length of multiple splines
     */
    public static double sumDCurvature2(List<QuinticHermiteSpline> splines) {
        double sum = 0;
        for (QuinticHermiteSpline s : splines) {
            sum += s.sumDCurvature2();
        }
        return sum;
    }

    /**
     * Makes optimization code a little more readable
     */
    private static class ControlPoint {
        private double ddx, ddy;
    }

    /**
     * Finds the optimal second derivative values for a set of splines to reduce the sum of the change in curvature
     * squared over the path
     *
     * @param splines the list of splines to optimize
     * @return the final sumDCurvature2
     */
    public static double optimizeSpline(List<QuinticHermiteSpline> splines) {
        int count = 0;
        double prev = sumDCurvature2(splines);
        while (count < kMaxIterations) {
            runOptimizationIteration(splines);
            double current = sumDCurvature2(splines);
            if (prev - current < kMinDelta)
                return current;
            prev = current;
            count++;
        }
        return prev;
    }


    /**
     * Runs a single optimization iteration
     */
    private static void runOptimizationIteration(List<QuinticHermiteSpline> splines) {
        //can't optimize anything with less than 2 splines
        if (splines.size() <= 1) {
            return;
        }

        ControlPoint[] controlPoints = new ControlPoint[splines.size() - 1];
        double magnitude = 0;

        for (int i = 0; i < splines.size() - 1; ++i) {
            //don't try to optimize colinear points
            if (splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            double original = sumDCurvature2(splines);
            QuinticHermiteSpline temp, temp1;

            temp = splines.get(i);
            temp1 = splines.get(i + 1);
            controlPoints[i] = new ControlPoint(); //holds the gradient at a control point

            //calculate partial derivatives of sumDCurvature2
            splines.set(i, new QuinticHermiteSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0, temp.ddx1 +
                    kEpsilon, temp.y0, temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1));
            splines.set(i + 1, new QuinticHermiteSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0 +
                    kEpsilon, temp1.ddx1, temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0, temp1.ddy1));
            controlPoints[i].ddx = (sumDCurvature2(splines) - original) / kEpsilon;
            splines.set(i, new QuinticHermiteSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0, temp.ddx1, temp
                    .y0, temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1 + kEpsilon));
            splines.set(i + 1, new QuinticHermiteSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0,
                    temp1.ddx1, temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0 + kEpsilon, temp1.ddy1));
            controlPoints[i].ddy = (sumDCurvature2(splines) - original) / kEpsilon;

            splines.set(i, temp);
            splines.set(i + 1, temp1);
            magnitude += controlPoints[i].ddx * controlPoints[i].ddx + controlPoints[i].ddy * controlPoints[i].ddy;
        }

        magnitude = Math.sqrt(magnitude);

        //minimize along the direction of the gradient
        //first calculate 3 points along the direction of the gradient
        Translation2d p1, p2, p3;
        p2 = new Translation2d(0, sumDCurvature2(splines)); //middle point is at the current location

        for (int i = 0; i < splines.size() - 1; ++i) { //first point is offset from the middle location by -stepSize
            if (splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            //normalize to step size
            controlPoints[i].ddx *= kStepSize / magnitude;
            controlPoints[i].ddy *= kStepSize / magnitude;

            //move opposite the gradient by step size amount
            splines.get(i).ddx1 -= controlPoints[i].ddx;
            splines.get(i).ddy1 -= controlPoints[i].ddy;
            splines.get(i + 1).ddx0 -= controlPoints[i].ddx;
            splines.get(i + 1).ddy0 -= controlPoints[i].ddy;

            //recompute the spline's coefficients to account for new second derivatives
            splines.get(i).computeCoefficients();
            splines.get(i + 1).computeCoefficients();
        }
        p1 = new Translation2d(-kStepSize, sumDCurvature2(splines));

        for (int i = 0; i < splines.size() - 1; ++i) { //last point is offset from the middle location by +stepSize
            if (splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            //move along the gradient by 2 times the step size amount (to return to original location and move by 1
            // step)
            splines.get(i).ddx1 += 2 * controlPoints[i].ddx;
            splines.get(i).ddy1 += 2 * controlPoints[i].ddy;
            splines.get(i + 1).ddx0 += 2 * controlPoints[i].ddx;
            splines.get(i + 1).ddy0 += 2 * controlPoints[i].ddy;

            //recompute the spline's coefficients to account for new second derivatives
            splines.get(i).computeCoefficients();
            splines.get(i + 1).computeCoefficients();
        }

        p3 = new Translation2d(kStepSize, sumDCurvature2(splines));

        double stepSize = fitParabola(p1, p2, p3); //approximate step size to minimize sumDCurvature2 along the gradient

        for (int i = 0; i < splines.size() - 1; ++i) {
            if (splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            //move by the step size calculated by the parabola fit (+1 to offset for the final transformation to find
            // p3)
            controlPoints[i].ddx *= 1 + stepSize / kStepSize;
            controlPoints[i].ddy *= 1 + stepSize / kStepSize;

            splines.get(i).ddx1 += controlPoints[i].ddx;
            splines.get(i).ddy1 += controlPoints[i].ddy;
            splines.get(i + 1).ddx0 += controlPoints[i].ddx;
            splines.get(i + 1).ddy0 += controlPoints[i].ddy;

            //recompute the spline's coefficients to account for new second derivatives
            splines.get(i).computeCoefficients();
            splines.get(i + 1).computeCoefficients();
        }
    }

    /**
     * fits a parabola to 3 points
     *
     * @return the x coordinate of the vertex of the parabola
     */
    private static double fitParabola(Translation2d p1, Translation2d p2, Translation2d p3) {
        double A = (p3.x() * (p2.y() - p1.y()) + p2.x() * (p1.y() - p3.y()) + p1.x() * (p3.y() - p2.y()));
        double B = (p3.x() * p3.x() * (p1.y() - p2.y()) + p2.x() * p2.x() * (p3.y() - p1.y()) + p1.x() * p1.x() *
                (p2.y() - p3.y()));
        return -B / (2 * A);
    }


    public double getOrientation(double t) {
        var theta =
                atheta * t * t * t * t * t +
                        btheta * t * t * t * t +
                        ctheta * t * t * t +
                        dtheta * t * t +
                        etheta * t +
                        ftheta;
        return theta;
    }

    /**
     * Subdivides a spline at point z [0,1]
     * @param z Split point [0,1]
     * @return List of two QuinticHermiteSpline
     */
    public List<QuinticHermiteSpline> subDivide(double z) {
        var point = getPoint(z);

        // Due to the change in parameterization, the derivatives need to be scaled appropriately.
        // To retain the shape of the spline segments the k-th derivative of the first  sub-segment
        // has to be scaled by z to the power of k, and the k-th derivative of the second sub-segment
        // by (1-z) to the power of k.

        List<QuinticHermiteSpline> segments = new ArrayList<>();
        segments.add(
                new QuinticHermiteSpline(
                        x0, point.x(), dx0 * z, dx(z) * z, ddx0 * z * z, ddx(z) * z * z,
                        y0, point.y(), dy0 * z, dy(z) * z, ddy0 * z * z, ddy(z) * z * z,
                        theta0, theta1, dtheta0, ddtheta1, ddtheta0, ddtheta1));
        segments.add(
                new QuinticHermiteSpline(
                        point.x(), x1, dx(z) * (1 - z), dx1 * (1 - z), ddx(z) * (1 - z) * (1 - z), ddx1 * (1 - z) * (1 - z),
                        point.y(), y1, dy(z) * (1 - z), dy1 * (1 - z), ddy(z) * (1 - z) * (1 - z), ddy1 * (1 - z) * (1 - z),
                        theta0, theta1, dtheta0, ddtheta1, ddtheta0, ddtheta1));
        return segments;
    }

    /**
     * Subdivides a spline at point z1, and z2 [0,1] where 0 < z1 <= z2 < 1.
     * @param z1    Split point [0,1]
     * @param z2    Split point [0,1] and z2 <= z1
     * @return      List of QuinticHermiteSpline
     */
    public List<QuinticHermiteSpline> subDivide3(double z1, double z2) {
        List<QuinticHermiteSpline> segments = new ArrayList<>();

        if ((z1 == 0.0 || z1 == 1.0) && z2 == 0.0) {
            segments.add(this);
            return segments;
        }

        if (z1 != 0.0 && z2 == 0.0) {
            return subDivide(z1);
        }

        if (z1 + (1 - z2) == 1.0) {
            return subDivide(z1);
        }

        if (z1 == 0.0) {
            return subDivide(z2);
        }

        var s1 = subDivide(z1);
        // Scale z given recently created sub-segment is shorter.
        var scale_factor = 1 - z1;

        var s2 = s1.get(1).subDivide(z2 * scale_factor);
        segments.add(s1.get(0));
        segments.addAll(s2);
        return segments;
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return String.format(
                "x0=%s y0=%s x1=%s, y1=%s, " +
                        "theta0=%s, theta1=%s, " +
                        "dtheta0=%s, dtheta1=%s, " +
                        "ddtheta0=%s, ddtheta1=%s",
                fmt.format(x0), fmt.format(y0),
                fmt.format(x1), fmt.format(y1),
                fmt.format(Math.toDegrees(theta0)), fmt.format(Math.toDegrees(theta1)),
                fmt.format(dtheta0), fmt.format(dtheta1),
                fmt.format(ddtheta0), fmt.format(ddtheta1));
    }
}

