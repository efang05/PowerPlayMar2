package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static java.lang.Math.toRadians;

import java.util.Arrays;
import java.util.List;
//89.222      1.00871
//89.3459     1.00732     avgX: 1.00759
//89.3966     1.006742

//103.03645   0.8734
//103.7182    0.8677      avgY: 0.870433
//103.4139    0.8702
/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 4.14233; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -2.25; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.01225; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.0175; // Multiplier in the Y direction

    public static double MIN_IMU_UPDATE_INTERVAL = 0.1;
    public static double MIN_STABLE_HEADING_TIME = 0.05;
    public static double HEADING_EPSILON = toRadians(0.5);

    private BNO055IMU imu;
    private double baseExtHeading;

    private ElapsedTime lastIMUUpdateTimer = new ElapsedTime();
    private ElapsedTime stableHeadingTimer = new ElapsedTime();
    private double stableCheckHeading;

    private List<Double> cachedWheelPositions = Collections.emptyList();
    private boolean useCachedWheelPositions = false;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private double getRawExternalHeading() {
        return Angle.norm(imu.getAngularOrientation().firstAngle);
    }

    private double getExternalHeading() {
        return Angle.norm(getRawExternalHeading() - baseExtHeading);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose) {
        baseExtHeading = Angle.norm(getRawExternalHeading() - pose.getHeading());

        super.setPoseEstimate(pose);
    }

    @Override
    public void update() {
        double currentHeading = getPoseEstimate().getHeading();
        // reset timer and stableCheckHeading if our heading has changed too much
        if (Math.abs(Angle.normDelta(currentHeading - stableCheckHeading)) > HEADING_EPSILON) {
            stableHeadingTimer.reset();
            stableCheckHeading = currentHeading;
        }

        if (lastIMUUpdateTimer.seconds() > MIN_IMU_UPDATE_INTERVAL
                && stableHeadingTimer.seconds() > MIN_STABLE_HEADING_TIME) {
            lastIMUUpdateTimer.reset();

            // load in latest wheel positions and update to apply to pose
            super.update();
            double extHeading = getExternalHeading();
            Pose2d pose = new Pose2d(getPoseEstimate().vec(), extHeading);
            super.setPoseEstimate(pose);

            // Don't update with new positions, but instead use previous (internally cached) wheel positions.
            // This ensures wheel movement isn't "lost" when the lastWheelPositions list (this list is internal to the
            // ThreeTrackingWheelLocalizer/super class) is emptied with our call to setPoseEstimate. Calling update
            // fills lastWheelPositions with the cached wheel positions and allows the later update calls to calculate a
            // movement rather than simply filling the empty lastWheelPositions list and not incrementing the pose.
            useCachedWheelPositions = true;
            super.update();
            useCachedWheelPositions = false;
        } else {
            super.update();
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        if (!useCachedWheelPositions || cachedWheelPositions.isEmpty()) {
            cachedWheelPositions = Arrays.asList(
                    encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                    encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                    encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
            );
        }
        return cachedWheelPositions;
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}