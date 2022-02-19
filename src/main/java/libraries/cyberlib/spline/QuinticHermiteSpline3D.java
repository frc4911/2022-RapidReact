package libraries.cyberlib.spline;

import org.ejml.simple.SimpleMatrix;

public class QuinticHermiteSpline3D extends Spline3D {
    private static SimpleMatrix hermiteBasis;
    private SimpleMatrix m_coefficients;

    public double[] xInitialControlVector;
    public double[] yInitialControlVector;
    public double[] zInitialControlVector;
    public double[] xFinalControlVector;
    public double[] yFinalControlVector;
    public double[] zFinalControlVector;


    public QuinticHermiteSpline3D(double[] xInitialControlVector, double[] xFinalControlVector,
                                  double[] yInitialControlVector, double[] yFinalControlVector) {
        this(xInitialControlVector, xFinalControlVector,
                yInitialControlVector, yFinalControlVector,
                new double[] {0,0,0}, new double[] {0,0,0});
    }

    /**
     * Constructs a quintic hermite spline with the specified control vectors.
     * Each control vector contains into about the location of the point, its
     * first derivative, and its second derivative.
     *
     * @param xInitialControlVector The control vector for the initial point in
     *                              the x dimension.
     * @param xFinalControlVector   The control vector for the final point in
     *                              the x dimension.
     * @param yInitialControlVector The control vector for the initial point in
     *                              the y dimension.
     * @param yFinalControlVector   The control vector for the final point in
     *                              the y dimension.
     * @param zInitialControlVector The control vector for the initial point in
     *                              the z dimension.
     * @param zFinalControlVector   The control vector for the final point in
     *                              the z dimension.
     */
    @SuppressWarnings("ParameterName")
    public QuinticHermiteSpline3D(double[] xInitialControlVector, double[] xFinalControlVector,
                                  double[] yInitialControlVector, double[] yFinalControlVector,
                                  double[] zInitialControlVector, double[] zFinalControlVector) {
        super(5);

        this.xInitialControlVector = xInitialControlVector;
        this.xFinalControlVector = xFinalControlVector;
        this.yInitialControlVector = yInitialControlVector;
        this.yFinalControlVector = yFinalControlVector;
        this.zInitialControlVector = zInitialControlVector;
        this.zFinalControlVector = zFinalControlVector;

        calculateCoefficients();
    }

    public void calculateCoefficients() {
        // Populate the coefficients for the actual spline equations.
        // Row 0 is x coefficients
        // Row 1 is y coefficients
        // Row 2 is z coefficients
        final var hermite = makeHermiteBasis();
        final var x = getControlVectorFromArrays(xInitialControlVector, xFinalControlVector);
        final var y = getControlVectorFromArrays(yInitialControlVector, yFinalControlVector);
        final var z = getControlVectorFromArrays(zInitialControlVector, zFinalControlVector);

        final var xCoeffs = (hermite.mult(x)).transpose();
        final var yCoeffs = (hermite.mult(y)).transpose();
        final var zCoeffs = (hermite.mult(z)).transpose();

        m_coefficients = new SimpleMatrix(9, 6);

        for (int i = 0; i < 6; i++) {
            m_coefficients.set(0, i, xCoeffs.get(0, i));
            m_coefficients.set(1, i, yCoeffs.get(0, i));
            m_coefficients.set(2, i, zCoeffs.get(0, i));
        }
        for (int i = 0; i < 6; i++) {
            // Populate Row 3, 4, and 5 with the derivatives of the equations above.
            // Here, we are multiplying by (5 - i) to manually take the derivative. The
            // power of the term in index 0 is 5, index 1 is 4 and so on. To find the
            // coefficient of the derivative, we can use the power rule and multiply
            // the existing coefficient by its power.
            m_coefficients.set(3, i, m_coefficients.get(0, i) * (5 - i));
            m_coefficients.set(4, i, m_coefficients.get(1, i) * (5 - i));
            m_coefficients.set(5, i, m_coefficients.get(2, i) * (5 - i));
        }
        for (int i = 0; i < 5; i++) {
            // Then populate row 6, 7 and 8 with the second derivatives.
            // Here, we are multiplying by (4 - i) to manually take the derivative. The
            // power of the term in index 0 is 4, index 1 is 3 and so on. To find the
            // coefficient of the derivative, we can use the power rule and multiply
            // the existing coefficient by its power.
            m_coefficients.set(6, i, m_coefficients.get(3, i) * (4 - i));
            m_coefficients.set(7, i, m_coefficients.get(4, i) * (4 - i));
            m_coefficients.set(8, i, m_coefficients.get(5, i) * (4 - i));
        }
    }

    /**
     * Returns the coefficients matrix.
     *
     * @return The coefficients matrix.
     */
    @Override
    protected SimpleMatrix getCoefficients() {
        return m_coefficients;
    }

    /**
     * Returns the hermite basis matrix for quintic hermite spline interpolation.
     *
     * @return The hermite basis matrix for quintic hermite spline interpolation.
     */
    private SimpleMatrix makeHermiteBasis() {
        if (hermiteBasis == null) {
            // Given P(i), P'(i), P''(i), P(i+1), P'(i+1), P''(i+1), the control
            // vectors, we want to find the coefficients of the spline
            // P(t) = a5 * t^5 + a4 * t^4 + a3 * t^3 + a2 * t^2 + a1 * t + a0.
            //
            // P(i)     = P(0)   = a0
            // P'(i)    = P'(0)  = a1
            // P''(i)   = P''(0) = 2 * a2
            // P(i+1)   = P(1)   = a5 + a4 + a3 + a2 + a1 + a0
            // P'(i+1)  = P'(1)  = 5 * a5 + 4 * a4 + 3 * a3 + 2 * a2 + a1
            // P''(i+1) = P''(1) = 20 * a5 + 12 * a4 + 6 * a3 + 2 * a2
            //
            // [ P(i)     ] = [  0  0  0  0  0  1 ][ a5 ]
            // [ P'(i)    ] = [  0  0  0  0  1  0 ][ a4 ]
            // [ P''(i)   ] = [  0  0  0  2  0  0 ][ a3 ]
            // [ P(i+1)   ] = [  1  1  1  1  1  1 ][ a2 ]
            // [ P'(i+1)  ] = [  5  4  3  2  1  0 ][ a1 ]
            // [ P''(i+1) ] = [ 20 12  6  2  0  0 ][ a0 ]
            //
            // To solve for the coefficients, we can invert the 6x6 matrix and move it
            // to the other side of the equation.
            //
            // [ a5 ] = [  -6.0  -3.0  -0.5   6.0  -3.0   0.5 ][ P(i)     ]
            // [ a4 ] = [  15.0   8.0   1.5 -15.0   7.0  -1.0 ][ P'(i)    ]
            // [ a3 ] = [ -10.0  -6.0  -1.5  10.0  -4.0   0.5 ][ P''(i)   ]
            // [ a2 ] = [   0.0   0.0   0.5   0.0   0.0   0.0 ][ P(i+1)   ]
            // [ a1 ] = [   0.0   1.0   0.0   0.0   0.0   0.0 ][ P'(i+1)  ]
            // [ a0 ] = [   1.0   0.0   0.0   0.0   0.0   0.0 ][ P''(i+1) ]
            hermiteBasis = new SimpleMatrix(6, 6, true, new double[]{
                    -06.0, -03.0, -00.5, +06.0, -03.0, +00.5,
                    +15.0, +08.0, +01.5, -15.0, +07.0, -01.0,
                    -10.0, -06.0, -01.5, +10.0, -04.0, +00.5,
                    +00.0, +00.0, +00.5, +00.0, +00.0, +00.0,
                    +00.0, +01.0, +00.0, +00.0, +00.0, +00.0,
                    +01.0, +00.0, +00.0, +00.0, +00.0, +00.0
            });
        }
        return hermiteBasis;
    }

    @Override
    public String toString() {
////        return m_coefficients.toString();


        final double x0 = xInitialControlVector[0];
        final double x1 = xFinalControlVector[0];
        final double y0 = yInitialControlVector[0];
        final double y1 = yFinalControlVector[0];
        final double z0 = zInitialControlVector[0];
        final double z1 = zFinalControlVector[0];

        final double dx0 = xInitialControlVector[1];
        final double dx1 = xFinalControlVector[1];
        final double dy0 = yInitialControlVector[1];
        final double dy1 = yFinalControlVector[1];
        final double dz0 = zInitialControlVector[1];
        final double dz1 = zFinalControlVector[1];

        final double ddx0 = xInitialControlVector[2];
        final double ddx1 = xFinalControlVector[2];
        final double ddy0 = yInitialControlVector[2];
        final double ddy1 = yFinalControlVector[2];
        final double ddz0 = zInitialControlVector[2];
        final double ddz1 = zFinalControlVector[2];

        return
            String.format("x0=%.3f, y0=%.3f, ", x0, y0) +
            String.format("x1=%.3f, y1=%.3f, ", x1, y1) +
            String.format("theta0=%.3f, theta1=%.3f, ", Math.toDegrees(z0), Math.toDegrees(z1)) +
            String.format("dtheta0=%.3f, dtheta1=%.3f, ", dz0, dz1) +
            String.format("ddtheta0=%.3f, ddtheta1=%.3f, ", ddz0, ddz1);
   }

    /**
     * Returns the control vector for each dimension as a matrix from the
     * user-provided arrays in the constructor.
     *
     * @param initialVector The control vector for the initial point.
     * @param finalVector   The control vector for the final point.
     * @return The control vector matrix for a dimension.
     */
    private SimpleMatrix getControlVectorFromArrays(double[] initialVector, double[] finalVector) {
        if (initialVector.length != 3 || finalVector.length != 3) {
            throw new IllegalArgumentException("Size of vectors must be 3");
        }
        return new SimpleMatrix(6, 1, true, new double[] {
                initialVector[0], initialVector[1], initialVector[2],
                finalVector[0], finalVector[1], finalVector[2]});
    }
}