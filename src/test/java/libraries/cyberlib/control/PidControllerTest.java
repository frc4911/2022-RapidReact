package libraries.cyberlib.control;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;


public class PidControllerTest {
    private static final double TEST_DT = 5.0e-3;
    private static final double kEpsilon = 1E-9;

    @Test
    public void setOutputRangeTest() {
        PidController pidController = new PidController(new PidGains(1.0, 0.0, 0.0));

        pidController.setSetpoint(5.0);
        assertEquals(5.0, pidController.calculate(0.0, TEST_DT), kEpsilon);

        pidController.setOutputRange(-1.0, 1.0);
        assertEquals(1.0, pidController.calculate(0.0, TEST_DT), kEpsilon);
        assertEquals(0.5, pidController.calculate(4.5, TEST_DT), kEpsilon);
        assertEquals(-1.0, pidController.calculate(10.0, TEST_DT), kEpsilon);
        assertEquals(-0.5, pidController.calculate(5.5, TEST_DT), kEpsilon);
    }

    @Test
    public void setOutputRangeVerifyArgumentsTest() {
        PidController pidController = new PidController(new PidGains(0.0, 0.0, 0.0));

        assertThrows(IllegalArgumentException.class, () -> {
            pidController.setOutputRange(1.0, -1.0);
        });
    }
}