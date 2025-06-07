package org.team340.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableInteger;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RioIO;
import org.team340.robot.util.ReefSelection;

@Logged
public final class Lights {

    private static final int LENGTH = 28;
    private static final int COUNT = 3;

    private static enum Color {
        LEVEL(255, 255, 255),
        HAS_CORAL(255, 255, 255),
        GOOSE(255, 255, 255),
        SCORED(0, 255, 0),
        GOOSE_ASSASSINATION(255, 0, 0),
        DISABLED(255, 32, 0),
        OFF(0, 0, 0);

        private final TunableInteger r;
        private final TunableInteger g;
        private final TunableInteger b;

        private Color(int r, int g, int b) {
            this.r = Tunable.value("lights/colors/" + name() + "/r", r);
            this.g = Tunable.value("lights/colors/" + name() + "/g", g);
            this.b = Tunable.value("lights/colors/" + name() + "/b", b);
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
     * Modifies the entire buffer to be a single color.
     * @param color The color to apply.
     */
    private void setAll(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, color.r(), color.g(), color.b());
        }
    }

    /**
     * Displays the disabled animation.
     */
    public Command disabled() {
        return Commands.startEnd(() -> setAll(Color.DISABLED), () -> setAll(Color.OFF), sides, top)
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
         * Displays the flames animation.
         */
        public Command flames() {
            int[] state = new int[LENGTH];
            for (int i = 0; i < state.length; i++) {
                state[i] = 0;
            }

            return commandBuilder()
                .onExecute(() -> {
                    for (int i = 0; i < LENGTH; i++) {
                        state[i] = (int) Math.max(
                            0.0,
                            state[i] - (Math2.random((0.5 + (i / (LENGTH * 0.11))) * 5.0) + 28.0)
                        );
                    }
                    for (int i = LENGTH - 1; i >= 2; i--) {
                        state[i] = (state[i - 1] + state[i - 2] + state[i - 2]) / 3;
                    }
                    if (Math.random() < 0.5) {
                        int i = (int) Math2.random(5.0);
                        state[i] = (int) (state[i] + Math2.random(160.0, 255.0));
                    }
                    for (int i = 0; i < LENGTH; i++) {
                        int heat = (int) ((state[i] / 255.0) * 191.0);
                        int ramp = (heat & 63) << 2;
                        if (heat > 180) {
                            setBoth(i, 255, 255, ramp);
                        } else if (heat > 60) {
                            setBoth(i, 255, ramp, 0);
                        } else {
                            setBoth(i, ramp, 0, 0);
                        }
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.flames()");
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
         * Displays the "Has Coral" animation.
         */
        public Command hasCoral(BooleanSupplier goosing, DoubleSupplier goosePosition, ReefSelection selection) {
            final double GOOSE_RANGE = 0.15;
            final double HALF_RANGE = GOOSE_RANGE / 2.0;

            return commandBuilder()
                .onExecute(() -> {
                    if (!goosing.getAsBoolean()) {
                        set(RobotController.getRSLState() ? Color.HAS_CORAL : Color.OFF);
                    } else {
                        double position = goosePosition.getAsDouble();
                        double percent =
                            (MathUtil.clamp(Math.abs(position), 0.5 - HALF_RANGE, 0.5 + HALF_RANGE) -
                                (0.5 - HALF_RANGE)) *
                            (1.0 / GOOSE_RANGE);
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
                })
                .onEnd(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Top.hasCoral()");
        }

        /**
         * Displays the "Scored" animation.
         */
        public Command scored() {
            Timer timer = new Timer();

            return commandBuilder()
                .onInitialize(() -> timer.restart())
                .onExecute(() -> set(timer.get() % 0.15 > 0.75 ? Color.SCORED : Color.OFF))
                .onEnd(() -> set(Color.OFF))
                .withTimeout(1.5)
                .ignoringDisable(true)
                .withName("Lights.Top.scored()");
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
