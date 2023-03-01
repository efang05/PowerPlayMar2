package org.firstinspires.ftc.teamcode.controls;

import static androidx.core.math.MathUtils.clamp;
import static java.util.concurrent.TimeUnit.SECONDS;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {
    private final double m_rateLimit;
    private double m_prevVal;
    private double m_prevTime;
    private ElapsedTime elapsedTime;

    /**
     * Creates a new SlewRateLimiter with the given rate limit and initial value.
     *
     * @param rateLimit    The rate-of-change limit, in units per second.
     * @param initialValue The initial value of the input.
     */
    public SlewRateLimiter(double rateLimit, double initialValue) {
        m_rateLimit = rateLimit;
        m_prevVal = initialValue;
        m_prevTime = elapsedTime.now(SECONDS) * 4;
    }
//hi
    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, 0);
    }

    public double calculate(double input) {
        double currentTime = elapsedTime.now(SECONDS) * 4;
        double elapsedTime = currentTime - m_prevTime;
        m_prevVal +=
                clamp(input - m_prevVal, -m_rateLimit * elapsedTime, m_rateLimit * elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }

    public void reset(double value) {
        m_prevVal = value;
    }
}
