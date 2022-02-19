package libraries.cyberlib.trajectory;

import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.trajectory.constraints.TrajectoryConstraint;

import java.util.ArrayList;
import java.util.List;

/**
 * Class used to parameterize an omnidirectional trajectory by time.
 */
public final class TrajectoryParameterizer {
    /**
     * Private constructor because this is a utility class.
     */
    private TrajectoryParameterizer() {
    }

    /**
     * Parameterize the trajectory by time. This is where the velocity profile is
     * generated.
     *
     * <p>The derivation of the algorithm used can be found
     * <a href="http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf"
     * here</a>.
     *
     * @param points                           Reference to the spline points.
     * @param constraints                      A vector of various velocity and acceleration.
     *                                         constraints.
     * @param startVelocityMetersPerSecond     The start translational velocity for the trajectory.
     * @param endVelocityMetersPerSecond       The end translational velocity for the trajectory.
     * @param maxVelocityMetersPerSecond       The max translational acceleration for the trajectory.
     * @param maxAccelerationMetersPerSecondSq The max translational acceleration for the trajectory.
     * @param startAngularVelocityRadiansPerSecond     The start angular velocity for the trajectory.
     * @param endAngularVelocityRadiansPerSecond       The end angular velocity for the trajectory.
     * @param maxAngularVelocityRadiansPerSecond       The max angular acceleration for the trajectory.
     * @param maxAngularAccelerationRadiansPerSecondSq The max angular acceleration for the trajectory.
     * @param reversed                         Whether the robot should move backwards.
     *                                         Note that the robot will still move from
     *                                         a -&gt; b -&gt; ... -&gt; z as defined in the
     *                                         waypoints.
     * @return The trajectory.
     */
    @SuppressWarnings({"PMD.ExcessiveMethodLength", "PMD.CyclomaticComplexity",
            "PMD.NPathComplexity", "PMD.AvoidInstantiatingObjectsInLoops"})
    public static Trajectory timeParameterizeTrajectory(
            List<PoseWithCurvatureAndOrientation> points,
            final List<TrajectoryConstraint> constraints,
            final double startVelocityMetersPerSecond,
            final double endVelocityMetersPerSecond,
            final double maxVelocityMetersPerSecond,
            final double maxAccelerationMetersPerSecondSq,
            final double startAngularVelocityRadiansPerSecond,
            final double endAngularVelocityRadiansPerSecond,
            final double maxAngularVelocityRadiansPerSecond,
            final double maxAngularAccelerationRadiansPerSecondSq,
            boolean reversed
    ) {
        var constrainedStates = new ArrayList<ConstrainedState>(points.size());
        var predecessor = new ConstrainedState(
                points.get(0),
                0,
                startVelocityMetersPerSecond,
                -maxAccelerationMetersPerSecondSq,
                maxAccelerationMetersPerSecondSq,
                startAngularVelocityRadiansPerSecond,
                -maxAngularAccelerationRadiansPerSecondSq,
                maxAngularAccelerationRadiansPerSecondSq);

        constrainedStates.add(predecessor);

        // Forward pass
        for (int i = 1; i < points.size(); i++) {
            constrainedStates.add(new ConstrainedState());
            var constrainedState = constrainedStates.get(i);
            constrainedState.pose = points.get(i);

            // Begin constraining based on predecessor.
            double ds =
                constrainedState
                    .pose
                    .poseMeters
                    .getTranslation()
                    .getDistance(predecessor.pose.poseMeters.getTranslation());
            constrainedState.distanceMeters = predecessor.distanceMeters + ds;

            constrainedState.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
            constrainedState.minAccelerationMetersPerSecondSq = -maxAccelerationMetersPerSecondSq;
            constrainedState.maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
            constrainedState.minAngularAccelerationRadiansPerSecondSq = -maxAngularAccelerationRadiansPerSecondSq;
            constrainedState.maxAngularAccelerationRadiansPerSecondSq = maxAngularAccelerationRadiansPerSecondSq;

            // Calculate max allowable velocity that meets translational and rotational bounds
            // The bounds can be treated as additional isolated constraints for v(i-1) and v(i)
            // and they are therefore easily integrated into the first phase of velocity profile
            // generation.

            // Calculate overlap for v(i-i)
            double vOverlap1 = calculateMaxAllowableVelocity(ds,
                    constrainedState.pose.curvatureRadPerMeter,
                    predecessor.pose.curvatureRadPerMeter,
                    maxAccelerationMetersPerSecondSq,
                    maxAngularAccelerationRadiansPerSecondSq);

            predecessor.maxVelocityMetersPerSecond =
                    Math.min(maxVelocityMetersPerSecond, vOverlap1);

            // Calculate overlap for v(i)
            var vOverlap2 = calculateMaxAllowableVelocity(ds,
                    predecessor.pose.curvatureRadPerMeter,
                    constrainedState.pose.curvatureRadPerMeter,
                    maxAccelerationMetersPerSecondSq,
                    maxAngularAccelerationRadiansPerSecondSq);

            constrainedState.maxVelocityMetersPerSecond =
                    Math.min(maxVelocityMetersPerSecond, vOverlap2);

//            System.out.printf("vOverlap1=%f, vOverlap2=%f %n", vOverlap1, vOverlap2);

            // We may need to iterate to find the maximum end velocity and common
            // acceleration, since acceleration limits may be a function of velocity.
            while (true) {

                // Enforce global max velocity and max reachable velocity by global
                // acceleration limit. vf = std::sqrt(vi^2 + 2*a*d).
                constrainedState.maxVelocityMetersPerSecond =
                    Math.min(
                            constrainedState.maxVelocityMetersPerSecond,
                            Math.sqrt(predecessor.maxVelocityMetersPerSecond * predecessor.maxVelocityMetersPerSecond
                                + predecessor.maxAccelerationMetersPerSecondSq * ds * 2.0)
                );

//                System.out.println(String.format("i = %d, ds = %f", i, ds));

//                System.out.printf("vOverlap=%f, constrained max=%f, predecessor %f%n", vOverlap, constrainedState.maxVelocityMetersPerSecond, predecessor.maxVelocityMetersPerSecond);

//                System.out.printf("(a)predecessor.maxVelocityMetersPerSecond=%.2f,%n", predecessor.maxVelocityMetersPerSecond);
//                System.out.printf("(b)predecessor.maxAccelerationMetersPerSecondSq=%f.2%n", predecessor.maxAccelerationMetersPerSecondSq);
//                System.out.printf("(c)global accel limit=%f%n", Math.sqrt(
//                        predecessor.maxVelocityMetersPerSecond * predecessor.maxVelocityMetersPerSecond
//                                + predecessor.maxAccelerationMetersPerSecondSq * ds * 2.0));
//                System.out.printf("(d)constrainedState.maxVelocityMetersPerSecond=%f.2%n", constrainedState.maxVelocityMetersPerSecond);


                // Limit rotational velocity based on first theta derivative at planning point s(i)
                double vTheta = constrainedState.maxAngularVelocityRadiansPerSecond < 1E-6 ?
                        Double.POSITIVE_INFINITY :
                        maxAngularVelocityRadiansPerSecond / Math.abs(constrainedState.maxAngularVelocityRadiansPerSecond);
//
//                constrainedState.maxVelocityMetersPerSecond =
//                        Math.min(constrainedState.maxVelocityMetersPerSecond, vTheta);

//                constrainedState.minAccelerationMetersPerSecondSq = -maxAccelerationMetersPerSecondSq;
//                constrainedState.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
//                constrainedState.minAngularAccelerationRadiansPerSecondSq = -maxAngularAccelerationRadiansPerSecondSq;
//                constrainedState.maxAngularAccelerationRadiansPerSecondSq = maxAngularAccelerationRadiansPerSecondSq;

                // Calculate max angular acceleration
//                constrainedState.maxAngularAccelerationRadiansPerSecondSq =
//                        Math.min(
//                                maxAngularAccelerationRadiansPerSecondSq,
//                                predecessor.pose.angularVelocity / ds);

//                constrainedState.maxAngularVelocityRadiansPerSecond =
//                        Math.min(
//                                maxAngularAccelerationRadiansPerSecondSq,
//                                Math.sqrt(predecessor.maxAngularVelocityRadiansPerSecond * predecessor.maxAngularVelocityRadiansPerSecond
//                                    + predecessor.maxAngularAccelerationRadiansPerSecondSq * ds * 2.0)
//                        );
                // At this point, the constrained state is fully constructed apart from
                // all the custom-defined user constraints.
                for (final var constraint : constraints) {
                    constrainedState.maxVelocityMetersPerSecond =
                        Math.min(
                            constrainedState.maxVelocityMetersPerSecond,
                            constraint.getMaxVelocity(
                                constrainedState.pose,
                                constrainedState.pose.curvatureRadPerMeter,
                                constrainedState.maxVelocityMetersPerSecond,
                                constrainedState.pose.angularVelocity));
                }

                // Now enforce all acceleration limits.
                enforceAccelerationLimits(reversed, constraints, constrainedState);

                if (ds < 1E-6) {
//                    System.out.println(String.format("i = %d, ds = %f", i, ds));
                    break;
                }

                // If the actual acceleration for this state is higher than the max
                // acceleration that we applied, then we need to reduce the max
                // acceleration of the predecessor and try again.
                double actualAcceleration =
                        (constrainedState.maxVelocityMetersPerSecond
                                    * constrainedState.maxVelocityMetersPerSecond
                                - predecessor.maxVelocityMetersPerSecond
                                    * predecessor.maxVelocityMetersPerSecond)
                        / (ds * 2.0);

//                System.out.printf("()actualAcceleration=%f%n", actualAcceleration);

                // If we violate the max acceleration constraint, let's modify the
                // predecessor.
                if (constrainedState.maxAccelerationMetersPerSecondSq < actualAcceleration - 1E-6) {
                    predecessor.maxAccelerationMetersPerSecondSq =
                        constrainedState.maxAccelerationMetersPerSecondSq;
                } else {
                    // Constrain the predecessor's max acceleration to the current
                    // acceleration.
                    if (actualAcceleration > predecessor.minAccelerationMetersPerSecondSq) {
                        predecessor.maxAccelerationMetersPerSecondSq = actualAcceleration;
                    }

                    // If the actual acceleration is less than the predecessor's min
                    // acceleration, it will be repaired in the backward pass.
                    break;
                }
            }
            predecessor = constrainedState;
        }

        var successor =
            new ConstrainedState(
                points.get(points.size() - 1),
                constrainedStates.get(constrainedStates.size() - 1).distanceMeters,
                endVelocityMetersPerSecond,
                -maxAccelerationMetersPerSecondSq,
                maxAccelerationMetersPerSecondSq,
                endAngularVelocityRadiansPerSecond,
                -maxAngularAccelerationRadiansPerSecondSq,
                maxAngularAccelerationRadiansPerSecondSq);

        // Backward pass
        for (int i = points.size() - 1; i >= 0; --i) {
            var constrainedState = constrainedStates.get(i);
            double ds = constrainedState.distanceMeters - successor.distanceMeters; // negative

            while (true) {
                // Enforce max velocity limit (reverse)
                // vf = std::sqrt(vi^2 + 2*a*d), where vi = successor.
                double newMaxVelocity = Math.sqrt(
                        successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond
                                + successor.minAccelerationMetersPerSecondSq * ds * 2.0
                );

                // Limit rotational velocity based on first theta derivative at planning point s(i)
//                double vTheta = successor.maxAngularVelocityRadiansPerSecond < 1E-6 ?
//                        Double.POSITIVE_INFINITY :
//                        maxAngularVelocityRadiansPerSecond / Math.abs(successor.maxAngularVelocityRadiansPerSecond);
//
//                newMaxVelocity = Math.min(newMaxVelocity, vTheta);

                // No more limits to impose! This state can be finalized.
                if (newMaxVelocity >= constrainedState.maxVelocityMetersPerSecond) {
                    break;
                }

                constrainedState.maxVelocityMetersPerSecond = newMaxVelocity;

                // Check all acceleration constraints with the new max velocity.
                enforceAccelerationLimits(reversed, constraints, constrainedState);

                if (ds > -1E-6) {
                    break;
                }

                // If the actual acceleration for this state is lower than the min
                // acceleration, then we need to lower the min acceleration of the
                // successor and try again.
                double actualAcceleration =
                    (constrainedState.maxVelocityMetersPerSecond
                            * constrainedState.maxVelocityMetersPerSecond
                            - successor.maxVelocityMetersPerSecond * successor.maxVelocityMetersPerSecond)
                            / (ds * 2.0);

                if (constrainedState.minAccelerationMetersPerSecondSq > actualAcceleration + 1E-6) {
                    successor.minAccelerationMetersPerSecondSq
                            = constrainedState.minAccelerationMetersPerSecondSq;
                } else {
                    successor.minAccelerationMetersPerSecondSq = actualAcceleration;
                    break;
                }
            }

            successor = constrainedState;
        }

        // Now we can integrate the constrained states forward in time to obtain our
        // trajectory states.
        var states = new ArrayList<Trajectory.State>(points.size());
        double timeSeconds = 0.0;
        double distanceMeters = 0.0;
        double velocityMetersPerSecond = 0.0;

        for (int i = 0; i < constrainedStates.size(); i++) {
            final var state = constrainedStates.get(i);

            // Calculate the change in position between the current state and the previous
            // state.
            double ds = state.distanceMeters - distanceMeters;

            // Calculate the acceleration between the current state and the previous
            // state.
            double accel = (state.maxVelocityMetersPerSecond * state.maxVelocityMetersPerSecond
                    - velocityMetersPerSecond * velocityMetersPerSecond) / (ds * 2);

            // Calculate dt
            double dt = 0.0;
            if (i > 0) {
                states.get(i - 1).accelerationMetersPerSecondSq = reversed ? -accel : accel;
                if (Math.abs(accel) > 1E-6) {
                    // v_f = v_0 + a * t
                    dt = (state.maxVelocityMetersPerSecond - velocityMetersPerSecond) / accel;
                } else if (Math.abs(velocityMetersPerSecond) > 1E-6) {
                    // delta_x = v * t
                    dt = ds / velocityMetersPerSecond;
                } else {
                    throw new TrajectoryGenerationException("Something went wrong at iteration " + i
                            + " of time parameterization.");
                }
            }

            velocityMetersPerSecond = state.maxVelocityMetersPerSecond;
            distanceMeters = state.distanceMeters;

            timeSeconds += dt;

            states.add(new Trajectory.State(
                    timeSeconds,
                    reversed ? -velocityMetersPerSecond : velocityMetersPerSecond,
                    reversed ? -accel : accel,
                    state.pose.poseMeters,
                    state.pose.curvatureRadPerMeter,
                    state.pose.orientation,
                    reversed ? -state.pose.angularVelocity : state.pose.angularVelocity,
                    reversed ? -state.pose.angularAcceleration : state.pose.angularAcceleration
            ));
        }

        return new Trajectory(states);
    }

    private static void enforceAccelerationLimits(
            boolean reverse,
            List<TrajectoryConstraint> constraints,
            ConstrainedState state) {

        for (final var constraint : constraints) {
            double factor = reverse ? -1.0 : 1.0;
            final var minMaxAccel = constraint.getMinMaxAcceleration(
                    state.pose,
                    state.pose.curvatureRadPerMeter,
                    state.maxVelocityMetersPerSecond * factor,
                    state.pose.angularVelocity);

            if (minMaxAccel.getMinAcceleration()
                    > minMaxAccel.getMaxAcceleration()) {
                throw new TrajectoryGenerationException("The constraint's min acceleration "
                        + "was greater than its max acceleration.\n Offending Constraint: "
                        + constraint.getClass().getName()
                        + "\n If the offending constraint was packaged with WPILib, please file a bug report.");
            }

            state.minAccelerationMetersPerSecondSq = Math.max(
                    state.minAccelerationMetersPerSecondSq,
                    reverse ? -minMaxAccel.getMaxAcceleration()
                            : minMaxAccel.getMinAcceleration());

            state.maxAccelerationMetersPerSecondSq = Math.min(
                    state.maxAccelerationMetersPerSecondSq,
                    reverse ? -minMaxAccel.getMaxAcceleration()
                            : minMaxAccel.getMinAcceleration());
        }
    }

    @SuppressWarnings("MemberName")
    private static class ConstrainedState {
        PoseWithCurvatureAndOrientation pose;
        double distanceMeters;
        double maxVelocityMetersPerSecond;
        double minAccelerationMetersPerSecondSq;
        double maxAccelerationMetersPerSecondSq;
        double maxAngularVelocityRadiansPerSecond;
        double minAngularAccelerationRadiansPerSecondSq;
        double maxAngularAccelerationRadiansPerSecondSq;

        ConstrainedState(PoseWithCurvatureAndOrientation pose, double distanceMeters,
                         double maxVelocityMetersPerSecond,
                         double minAccelerationMetersPerSecondSq,
                         double maxAccelerationMetersPerSecondSq,
                         double maxAngularVelocityRadiansPerSecond,
                         double minAngularAccelerationRadiansPerSecondSq,
                         double maxAngularAccelerationRadiansPerSecondSq) {
            this.pose = pose;
            this.distanceMeters = distanceMeters;
            this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
            this.minAccelerationMetersPerSecondSq = minAccelerationMetersPerSecondSq;
            this.maxAccelerationMetersPerSecondSq = maxAccelerationMetersPerSecondSq;
            this.maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;
            this.minAngularAccelerationRadiansPerSecondSq = minAngularAccelerationRadiansPerSecondSq;
            this.maxAngularAccelerationRadiansPerSecondSq = maxAngularAccelerationRadiansPerSecondSq;
        }

        ConstrainedState() {
            pose = new PoseWithCurvatureAndOrientation();
        }
    }

    public static class TrajectoryGenerationException extends RuntimeException {
        public TrajectoryGenerationException(String message) {
            super(message);
        }
    }

    /**
     * Computes the upper bound for v(i-1) that guarantees overlap of
     * [v(min)|a(trans), v(max)|a(trans)] and I(a(rot)) at a given
     * planning point p(i) for all smaller values.
     *
     * <p>This algorithm is derived from the pseudo code in Section B.2. of
     * C. Sprunk's, 2008  <a href="http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf">
     * Planning Motion Trajectories for Mobile Robots Using Splines</a>.
     *
     * <p>See section 3.2.6 for the discussion of translational and rotational acceleration constraints, and
     * section 3 2.7 satisfying them by solving the overlap problem.
     * The complete analysis can be seen in Appendix B.
     *
     * @param ds          Arch length between planning points p(i-1) and p(i)
     * @param c_cur       Curvature at p(i)
     * @param c_prev      Curvature at p(i-1)
     * @param a_trans_max Maximum translational acceleration
     * @param a_rot_max   Maximum rotational acceleration
     * @return  The upper bound for the translational velocity
     */
    public static double calculateMaxAllowableVelocity(
            final double ds,
            final double c_cur,
            final double c_prev,
            final double a_trans_max,
            final double a_rot_max) {

        double thresh = Double.POSITIVE_INFINITY;

        // Case 1
        if ((c_cur > 0.0) && (c_prev >= 0.0)) {
            // Case 1.1
            if (c_cur > c_prev) {
//                System.out.println("Case 1.1");
                thresh = Math.sqrt(
                        (2 * ds * Math.pow((a_rot_max + (c_cur * a_trans_max)), 2)) /
                        ((a_trans_max * (c_cur + c_prev) + (2 * a_rot_max)) * (c_cur - c_prev)));
            }

            // Case 1.2
            if (c_cur < c_prev) {
//                System.out.println("Case 1.2");
                double thresh1 = Math.sqrt(
                        (8 * c_cur * a_rot_max * ds) /
                                ((c_prev + c_cur) * (c_prev + c_cur)));
                double tmp1 = Math.sqrt(
                        (4 * c_cur * ds * (c_cur * a_trans_max + a_rot_max)) /
                                ((c_prev - c_cur) * (c_prev - c_cur)));
                double tmp2 = Math.sqrt(
                        (2 * ds * Math.pow((c_cur * a_trans_max + a_rot_max), 2)) /
                                ((c_prev - c_cur) * (2 * a_rot_max + (c_prev + c_cur) * a_trans_max)));
                double thresh_tmp1 = Math.min(tmp1, tmp2);
                double thresh_tmp2 = Math.min(
                        Math.sqrt(2 * a_rot_max * ds / c_prev),
                        Math.sqrt(2 * a_trans_max * ds));
                double thresh_tmp3 = Double.NEGATIVE_INFINITY;
                double tmp = Math.min(
                        (2 * a_rot_max * ds / c_prev),
                        (2 * ds * Math.pow((c_cur * a_trans_max - a_rot_max), 2)) /
                                ((c_prev - c_cur) * (2 * a_rot_max - (c_prev + c_cur) * a_trans_max)));
                if ((tmp > ((-4 * c_cur * ds * (c_cur * a_trans_max - a_rot_max)) / ((c_prev - c_cur) * (c_prev + c_cur))))
                        && (tmp > 2 * a_trans_max * ds)) {
                    thresh_tmp3 = Math.sqrt(tmp);
                }
                thresh = Math.max(Math.max(thresh1, thresh_tmp1), Math.max(thresh_tmp2, thresh_tmp3));
            }

            // Case 1.3
            if (c_cur == c_prev) {
//                System.out.println("Case 1.3");
                thresh = Double.POSITIVE_INFINITY;
            }
        }
        // Case 2
        if ((c_cur < 0.0) && (c_prev <= 0.0)) {
            // Case 2.1
            if (c_cur > c_prev) {
//                System.out.println("Case 2.1");
                double thresh1 = Math.sqrt(
                        (-8 * c_cur * a_rot_max * ds) /
                                ((c_prev + c_cur) * (c_prev + c_cur)));
                double tmp1 = Math.sqrt(
                        (-4 * c_cur * ds * (a_rot_max - c_cur * a_trans_max))/
                                ((c_prev + c_cur) * (c_prev - c_cur)));
                double tmp2 = Math.sqrt(
                        (-2 * ds * Math.pow((a_rot_max - c_cur * a_trans_max), 2)) /
                                ((c_prev - c_cur) * (2 * a_rot_max - (c_prev + c_cur) * a_trans_max)));
                double thresh_tmp1 = Math.min(tmp1, tmp2);
                double thresh_tmp2 = Math.min(
                        Math.sqrt(-2 * a_rot_max * ds / c_prev),
                        Math.sqrt(2 * a_trans_max * ds));
                double thresh_tmp3 = Double.NEGATIVE_INFINITY;
                double tmp = Math.min(
                        (-2 * a_rot_max * ds) / c_prev,
                        (-2 * ds * Math.pow((a_rot_max + c_cur * a_trans_max), 2)) /
                                ((c_prev - c_cur) * (2 * a_rot_max + (c_prev + c_cur) * a_trans_max)));
                if ((tmp > -4 * c_cur * ds * (a_trans_max + c_cur * a_rot_max) / ((c_prev - c_cur) * (c_prev + c_cur)))
                        && (tmp > 2 * a_trans_max * ds)) {
                    thresh_tmp3 = Math.sqrt(tmp);
                }

                thresh = Math.max(Math.max(thresh1, thresh_tmp1), Math.max(thresh_tmp2, thresh_tmp3));
            }

            // Case 2.2
            if (c_cur < c_prev) {
//                System.out.println("Case 2.2");
                thresh = Math.sqrt(
                        (-2 * ds * Math.pow((a_rot_max - c_cur * a_trans_max), 2)) /
                                ((c_prev - c_cur) * ((c_cur + c_prev) * a_trans_max - 2 * a_rot_max)));
            }

            // Case 2.3
            if (c_cur == c_prev) {
//                System.out.println("Case 2.3");
                thresh = Double.POSITIVE_INFINITY;
            }
        }

        // case 3
        if ((c_cur < 0.0) && (c_prev > 0.0)) {
//            System.out.println("Case 3");
            double vtwostarpos = Math.sqrt(2 * ds * a_rot_max / c_prev);
            double precond = Double.POSITIVE_INFINITY;
            if ((c_prev + c_cur) < 0) {
                precond = Math.sqrt(
                        (-4 * c_cur * ds * (c_cur * a_trans_max - a_rot_max)) /
                                ((c_prev - c_cur) * (c_prev + c_cur)));
            }

            double thresh_tmp = Math.min(
                    precond,
                    Math.sqrt(
                            (-2 * ds * Math.pow((c_cur * a_trans_max - a_rot_max), 2)) /
                                ((c_prev - c_cur) * ((c_prev + c_cur) * a_trans_max + 2 * a_rot_max))));

            thresh_tmp = Math.max(thresh_tmp, Math.sqrt(2 * ds * a_trans_max));
            thresh = Math.min(thresh_tmp, vtwostarpos);
        }

        // case 4
        if ((c_cur > 0.0)  && (c_prev < 0.0)) {
//            System.out.println("Case 4");
            double vonestarpos = Math.sqrt(-2 * ds * a_rot_max / c_prev);
            double precond = Double.POSITIVE_INFINITY;
            if ((c_prev + c_prev) > 0) {
                precond = Math.sqrt(
                        (-4 * c_cur * ds * (a_rot_max + c_cur * a_trans_max)) /
                                ((c_prev - c_cur) * (c_prev + c_cur)));
            }
            double thresh_tmp = Math.min(precond,
                    Math.sqrt(
                            (-2 * ds * Math.pow((a_rot_max + c_cur * a_trans_max), 2)) /
                                ((c_prev - c_cur) * ((c_prev + c_cur) * a_trans_max + 2 * a_rot_max))));

            thresh_tmp = Math.max(thresh_tmp, Math.sqrt(2 * ds * a_trans_max));
            thresh = Math.min(thresh_tmp, vonestarpos);
        }

        // case #5
        if (c_cur == 0.0) {
            // Case 5.1
            if (c_prev > 0) {
//                System.out.println("Case 5.1");
                double vtwohatpos  = Math.sqrt(2 * ds * a_rot_max / c_prev);
                double thresh_tmp = Math.max(
                        Math.sqrt(2 * ds * a_trans_max),
                        Math.sqrt(
                                (-2 * ds * a_rot_max * a_rot_max) /
                                        (c_prev * ((c_prev * a_trans_max) - 2 * a_rot_max))));
                thresh = Math.min(vtwohatpos, thresh_tmp);
            }

            // Case 5.2
            if (c_prev < 0) {
//                System.out.println("Case 5.2");
                double vonehatpos = Math.sqrt(-2 * ds * a_rot_max / c_prev);
                double thresh_tmp = Math.max(
                        Math.sqrt(2 * ds * a_trans_max),
                        Math.sqrt(
                                (-2 * ds * a_rot_max * a_rot_max) /
                                        (c_prev * ((c_prev * a_trans_max) + 2 * a_rot_max))));
                thresh = Math.min(vonehatpos, thresh_tmp);
            }
        }

        // Case 6
        if ((c_cur == 0.0) && (c_prev == 0.0)) {
//             System.out.println("Case 6");
            thresh = Double.POSITIVE_INFINITY;
        }

//        System.out.println(String.format("thresh {%.2f}", thresh));
        return thresh;
    }
}