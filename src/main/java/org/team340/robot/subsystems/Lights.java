package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.math.Math2;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableInteger;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RioIO;
import org.team340.robot.util.ReefSelection;

@Logged
public final class Lights {

    private static final int LENGTH = 28;
    private static final int COUNT = 3;

    private static final TunableTable tunables = Tunables.getNested("lights");

    private static enum Color {
        LEVEL(255, 255, 255),
        HAS_CORAL(255, 255, 255),
        GOOSE(255, 255, 255),
        SCORED(0, 255, 0),
        GOOSE_ASSASSINATION(255, 0, 0),
        BLUE(0, 10, 255),
        RED(255, 0, 0),
        NO_DS(255, 0, 255),
        NO_TAGS(255, 0, 0),
        BAD_LOCATION(255, 144, 0),
        BAD_ROTATION(255, 32, 0),
        OFF(0, 0, 0);

        private final TunableInteger r;
        private final TunableInteger g;
        private final TunableInteger b;

        private Color(int r, int g, int b) {
            this.r = tunables.value(name() + "/r", r);
            this.g = tunables.value(name() + "/g", g);
            this.b = tunables.value(name() + "/b", b);
        }

        private int r() {
            return r.get();
        }

        private int g() {
            return g.get();
        }

        private int b() {
            return b.get();
        }
    }

    public final Sides sides;
    public final Top top;

    private final AddressableLED lights;
    private final AddressableLEDBuffer buffer;

    public Lights() {
        lights = new AddressableLED(RioIO.LIGHTS);
        buffer = new AddressableLEDBuffer(LENGTH * COUNT);

        lights.setLength(buffer.getLength());
        lights.start();

        sides = new Sides();
        top = new Top();
    }

    public void update() {
        lights.setData(buffer);
    }

    /**
     * Displays the pre-match animation.
     * @param robotPose The robot's current pose.
     * @param seesAprilTag If the robot has seen an AprilTag since the last loop.
     * @param defaultAuto If the default auto is selected.
     */
    public Command preMatch(Supplier<Pose2d> robotPose, BooleanSupplier seesAprilTag, BooleanSupplier defaultAuto) {
        return parallel(sides.preMatch(defaultAuto), top.preMatch(robotPose, seesAprilTag))
            .until(DriverStation::isEnabled)
            .ignoringDisable(true)
            .withName("Lights.disabled()");
    }

    @Logged
    public final class Sides extends GRRSubsystem {

        private Sides() {}

        /**
         * Modifies the entire side LED strips to be a single color.
         * @param color The color to apply.
         */
        private void setBoth(Color color) {
            for (int i = 0; i < LENGTH; i++) {
                setBoth(i, color);
            }
        }

        /**
         * Modifies the buffer with values mirrored across the "center" of the LED strip.
         * @param i The index of the buffer to modify.
         * @param color The color to apply.
         */
        private void setBoth(int i, Color color) {
            setBoth(i, color.r(), color.g(), color.b());
        }

        /**
         * Modifies the buffer with values mirrored across the "center" of the LED strip.
         * @param i The index of the buffer to modify.
         * @param r Red value from {@code 0} to {@code 255}.
         * @param g Green value from {@code 0} to {@code 255}.
         * @param b Blue value from {@code 0} to {@code 255}.
         */
        private void setBoth(int i, int r, int g, int b) {
            setSingle(false, i, r, g, b);
            setSingle(true, i, r, g, b);
        }

        /**
         * Sets a single side's LED to a specified color.
         * @param left {@code true} for the left side, {@code false} for the right side.
         * @param i The index of the strip.
         * @param color The color to apply.
         */
        private void setSingle(boolean left, int i, Color color) {
            setSingle(left, i, color.r(), color.g(), color.b());
        }

        /**
         * Sets a single side's LED to a specified color.
         * @param left {@code true} for the left side, {@code false} for the right side.
         * @param i The index of the strip.
         * @param r Red value from {@code 0} to {@code 255}.
         * @param g Green value from {@code 0} to {@code 255}.
         * @param b Blue value from {@code 0} to {@code 255}.
         */
        private void setSingle(boolean left, int i, int r, int g, int b) {
            if (i >= LENGTH) return;
            buffer.setRGB(!left ? i : buffer.getLength() - i - 1, r, g, b);
        }

        /**
         * Displays the currently selected reef level.
         * @param selection The reef selection.
         */
        public Command levelSelection(ReefSelection selection) {
            return commandBuilder()
                .onExecute(() -> {
                    for (int i = 0; i < LENGTH; i++) {
                        for (int j = 0; j <= 1; j++) {
                            boolean blink = selection.isScoring() && (selection.isLeft() ? 0 : 1) == j;

                            if (
                                (blink ? RobotController.getRSLState() : true) && i < ((28 / 4) * selection.getLevel())
                            ) {
                                setSingle(j == 0, i, Color.LEVEL);
                            } else {
                                setSingle(j == 0, i, Color.OFF);
                            }
                        }
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.levelSelection()");
        }

        /**
         * Displays the percentage of the climber deploy position.
         * @param percent The climber's distance from the deploy position as a percent from [0.0, 1.0].
         */
        public Command climbPercent(DoubleSupplier percent) {
            return commandBuilder()
                .onExecute(() -> {
                    for (int i = 0; i < LENGTH; i++) {
                        setBoth(
                            i,
                            i > Math.ceil(percent.getAsDouble() * LENGTH)
                                ? Color.OFF
                                : Alliance.isBlue()
                                    ? Color.BLUE
                                    : Color.RED
                        );
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.climbPercent()");
        }

        /**
         * Displays the flames animation.
         */
        public Command flames(boolean allianceColors) {
            Mutable<Integer> count = new Mutable<>(0);
            int[] state = new int[LENGTH];

            return commandBuilder()
                .onInitialize(() -> {
                    count.value = 0;
                    for (int i = 0; i < state.length; i++) {
                        state[i] = 0;
                    }
                })
                .onExecute(() -> {
                    for (int i = 0; i < LENGTH; i++) {
                        state[i] = Math.max(0, state[i] - 16);
                    }

                    for (int i = LENGTH - 1; i >= 2; i--) {
                        state[i] = (state[i - 1] + state[i - 2] + state[i - 2]) / 3;
                    }

                    if (count.value++ % 2 == 0) {
                        int i = (int) Math2.random(3.0);
                        state[i] = (int) Math.min(255.0, state[i] + Math2.random(200.0, 255.0));
                    }

                    for (int i = 0; i < LENGTH; i++) {
                        int heat = Math.min(255, state[i] + (Math.random() < 0.05 && state[i] > 16 ? 32 : 0));
                        int ramp = (((int) Math.round((heat / 255.0) * 191.0)) & 63) << 2;
                        if (heat > 170) {
                            if (allianceColors) {
                                if (Alliance.isBlue()) setBoth(i, ramp, ramp, 255);
                                else setBoth(i, 255, ramp, ramp);
                            } else {
                                setBoth(i, 255, 255, ramp / 2);
                            }
                        } else if (heat > 84) {
                            if (allianceColors) {
                                if (Alliance.isBlue()) setBoth(i, ramp / 4, 0, 255);
                                else setBoth(i, 255, 0, ramp / 4);
                            } else {
                                setBoth(i, 255, ramp / 2, 0);
                            }
                        } else {
                            if (allianceColors && Alliance.isBlue()) {
                                setBoth(i, 0, 0, ramp);
                            } else {
                                setBoth(i, ramp, 0, 0);
                            }
                        }
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.flames()");
        }

        /**
         * Displays the pre-match animation.
         * @param defaultAuto If the default auto is selected.
         */
        public Command preMatch(BooleanSupplier defaultAuto) {
            Timer timer = new Timer();

            return commandBuilder()
                .onExecute(() -> {
                    if (defaultAuto.getAsBoolean()) {
                        timer.stop();
                        timer.reset();
                    } else {
                        timer.start();
                    }

                    if (timer.get() % 0.12 > 0.06 || timer.hasElapsed(1.0)) {
                        setBoth(Alliance.isBlue() ? Color.BLUE : Color.RED);
                    } else {
                        setBoth(Color.OFF);
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.preMatch()");
        }

        /**
         * Turns the lights off.
         */
        public Command off() {
            return commandBuilder()
                .onInitialize(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.off()");
        }
    }

    @Logged
    public final class Top extends GRRSubsystem {

        private Top() {}

        /**
         * Modifies the entire side LED strips to be a single color.
         * @param color The color to apply.
         */
        private void set(Color color) {
            for (int i = 0; i < LENGTH; i++) set(i, color);
        }

        private void set(int i, Color color) {
            if (i >= LENGTH) return;
            buffer.setRGB(LENGTH + i, color.r(), color.g(), color.b());
        }

        /**
         * Displays the coral state.
         */
        public Command coralDisplay(
            BooleanSupplier hasCoral,
            BooleanSupplier goosing,
            DoubleSupplier goosePosition,
            ReefSelection selection
        ) {
            final double GOOSE_RANGE = 0.15;
            final double HALF_RANGE = GOOSE_RANGE / 2.0;

            Timer timer = new Timer();

            return sequence(
                run(() -> set(Color.OFF)).until(hasCoral::getAsBoolean),
                run(() -> {
                    if (!goosing.getAsBoolean()) {
                        set(RobotController.getRSLState() ? Color.HAS_CORAL : Color.OFF);
                    } else {
                        double position = goosePosition.getAsDouble();
                        double percent =
                            (MathUtil.clamp(Math.abs(position), 0.5 - HALF_RANGE, 0.5 + HALF_RANGE)
                                - (0.5 - HALF_RANGE))
                            * (1.0 / GOOSE_RANGE);
                        if (position < 0.0) percent = 1.0 - percent;
                        int closestLED = (int) Math.round(percent * (LENGTH - 1));
                        for (int i = 0; i < LENGTH; i++) {
                            if (Math.abs(closestLED - i) <= 1) {
                                set(i, Color.GOOSE);
                            } else {
                                set(i, Color.OFF);
                            }
                        }
                    }
                }).until(() -> !hasCoral.getAsBoolean()),
                run(() -> set(timer.get() % 0.2 > 0.1 ? Color.SCORED : Color.OFF))
                    .beforeStarting(timer::restart)
                    .withTimeout(1.5)
            )
                .repeatedly()
                .ignoringDisable(true)
                .withName("Lights.Top.coralDisplay()");
        }

        /**
         * Displays that the goose has been killed.
         */
        public Command gooseAssassination() {
            return commandBuilder()
                .onInitialize(() -> set(Color.GOOSE_ASSASSINATION))
                .onEnd(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Top.gooseAssassination()");
        }

        /**
         * Displays the pre-match animation.
         * @param robotPose The robot's current pose.
         * @param seesAprilTag If the robot has seen an AprilTag since the last loop.
         */
        public Command preMatch(Supplier<Pose2d> robotPose, BooleanSupplier seesAprilTag) {
            final double LOCATION_TOL = 0.05;
            final double ROTATION_TOL = Math.toRadians(0.4);

            Debouncer tag = new Debouncer(0.5, DebounceType.kFalling);
            Debouncer location = new Debouncer(0.5, DebounceType.kFalling);
            Debouncer rotation = new Debouncer(0.5, DebounceType.kFalling);

            Debouncer sightDebounce = new Debouncer(0.06, DebounceType.kFalling);
            SlewRateLimiter center = new SlewRateLimiter(ROTATION_TOL * 2.0);
            Mutable<Pose2d> lastPose = new Mutable<>(Pose2d.kZero);
            Mutable<Double> errorTime = new Mutable<>(-1.0);
            List<Color> errors = new ArrayList<>();

            return commandBuilder()
                .onInitialize(() -> {
                    Pose2d pose = robotPose.get();
                    center.reset(pose.getRotation().getRadians());
                    lastPose.value = pose;
                    errorTime.value = -1.0;
                })
                .onExecute(() -> {
                    if (!DriverStation.isDSAttached()) {
                        set(Color.NO_DS);
                        return;
                    }

                    errors.clear();
                    Pose2d pose = robotPose.get();

                    if (tag.calculate(!sightDebounce.calculate(seesAprilTag.getAsBoolean()))) {
                        errors.add(Color.NO_TAGS);
                    } else if (
                        location.calculate(
                            !Math2.isNear(lastPose.value.getTranslation(), pose.getTranslation(), LOCATION_TOL)
                        )
                    ) {
                        errors.add(Color.BAD_LOCATION);
                    } else if (
                        rotation.calculate(
                            !Math2.isNear(lastPose.value.getRotation(), pose.getRotation(), ROTATION_TOL)
                        )
                    ) {
                        errors.add(Color.BAD_ROTATION);
                    }

                    lastPose.value = pose;
                    if (!errors.isEmpty()) {
                        double now = Timer.getFPGATimestamp();
                        if (errorTime.value < 0.0) errorTime.value = now;

                        double period = (now - errorTime.value) % 0.2;
                        if (period < 0.15) {
                            int i = (int) Math.floor((period / 0.15) * errors.size());
                            set(errors.get(Math.min(i, errors.size())));
                        } else {
                            set(Color.OFF);
                        }
                    } else {
                        double radians = pose.getRotation().getRadians();

                        if (errorTime.value > 0.0) {
                            errorTime.value = -1.0;
                            center.reset(radians);
                        }

                        double view = Math.toRadians(1.0);
                        double percent = (center.calculate(radians) + (view / 2.0) - radians) / view;
                        int closestLED = (int) Math.round(percent * (LENGTH - 1));
                        Color alliance = Alliance.isBlue() ? Color.BLUE : Color.RED;
                        for (int i = 0; i < LENGTH; i++) {
                            if (Math.abs(closestLED - i) <= 1) {
                                set(i, alliance);
                            } else {
                                set(i, Color.OFF);
                            }
                        }
                    }
                })
                .onEnd(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Top.preMatch()");
        }

        /**
         * Displays the climbing animation.
         */
        public Command climbing(BooleanSupplier isDeployed) {
            return commandBuilder()
                .onExecute(() ->
                    set(
                        !RobotController.getRSLState() && isDeployed.getAsBoolean()
                            ? Color.OFF
                            : (Alliance.isBlue() ? Color.BLUE : Color.RED)
                    )
                )
                .onEnd(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.climbing()");
        }

        /**
         * Turns the lights off.
         */
        public Command off() {
            return commandBuilder()
                .onInitialize(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Top.off()");
        }
    }
}
