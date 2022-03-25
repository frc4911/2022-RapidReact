package libraries.cyberlib.paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class PathTest {
    @Test
    void parameterize_line_1() {
        var builder = new PathBuilder(new Translation2d(), new Rotation2d())
                .lineTo(new Translation2d(1.0, 1.0));

        var points = builder.build().parameterize();
        for (var p : points) {
            System.out.println(p.toString());
        }
    }

    @Test
    void parameterize_line_2() {
        var builder = new PathBuilder(new Translation2d(), new Rotation2d())
                .lineTo(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(180));

        var points = builder.build().parameterize();
        for (var p : points) {
            System.out.println(p.toString());
        }
    }

    @Test
    void parameterize_line_3() {
        var builder = new PathBuilder(new Translation2d(), new Rotation2d())
                .lineTo(new Translation2d(1.0, 1.0),
                        new Translation2d(2.0, 0.0));

        var points = builder.build().parameterize();
        for (var p : points) {
            System.out.println(p.toString());
        }
    }

    @Test
    void parameterize_arc_1() {
        var builder = new PathBuilder(new Translation2d(), new Rotation2d())
                .arcTo(new Translation2d(1.0, 1.0),
                        new Translation2d(1.0, 0.0));

        var points = builder.build().parameterize();
        for (var p : points) {
            System.out.println(p.toString());
        }
    }

    @Test
    void parameterize_arc_2() {
        var builder = new PathBuilder(new Translation2d(), new Rotation2d())
                .arcTo(new Translation2d(1.0, 1.0),
                        new Translation2d(1.0, 0.0),
                        Rotation2d.fromDegrees(180));

        var points = builder.build().parameterize();
        for (var p : points) {
            System.out.println(p.toString());
        }
    }

    @Test
    void parameterize_arc_and_line() {
        var builder = new PathBuilder(new Translation2d(), new Rotation2d())
                .arcTo(new Translation2d(1.0, 1.0),
                        new Translation2d(1.0, 0.0),
                        Rotation2d.fromDegrees(-90))
                .lineTo(new Translation2d(2.0, 1.0));

        var points = builder.build().parameterize();
        for (var p : points) {
            System.out.println(p.toString());
        }
    }

    @Test
    void parameterize_spline() {
        var builder = new PathBuilder(new Translation2d(), new Rotation2d())
                .splineTo(new Translation2d(1.0, 1.0));

        var points = builder.build().parameterize();
        for (var p : points) {
            System.out.println(p.toString());
        }
    }
}
