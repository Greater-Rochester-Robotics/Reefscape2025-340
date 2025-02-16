package org.team340.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableInteger;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RioIO;

@Logged
public class Lights {

    private static enum Color {
        kLx(255, 255, 255),
        kScored(0, 255, 0),
        kHasCoral(255, 255, 255),
        kDisabled(255, 11, 0),
        kOff(0, 0, 0);

        private final TunableInteger r;
        private final TunableInteger g;
        private final TunableInteger b;

        private Color(int r, int g, int b) {
            this.r = Tunable.intValue("lights/colors/" + name() + "/r", r);
            this.g = Tunable.intValue("lights/colors/" + name() + "/g", g);
            this.b = Tunable.intValue("lights/colors/" + name() + "/b", b);
        }

        private int r() {
            return r.value();
        }

        private int g() {
            return g.value();
        }

        private int b() {
            return b.value();
        }
    }

    private static final int kStripLength = 28;
    private static final int kStripCount = 3;

    public final Sides sides;
    public final Top top;

    private final AddressableLED lights;
    private final AddressableLEDBuffer buffer;

    public Lights() {
        lights = new AddressableLED(RioIO.kLights);
        buffer = new AddressableLEDBuffer(kStripLength * kStripCount);

        lights.setLength(buffer.getLength());
        lights.start();

        sides = new Sides();
        top = new Top();

        setAll(Color.kDisabled);
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
        return Commands.startEnd(() -> setAll(Color.kDisabled), () -> setAll(Color.kOff), sides, top)
            .ignoringDisable(true)
            .withName("Lights.disabled()");
    }

    @Logged
    public class Sides extends GRRSubsystem {

        private Sides() {}

        /**
         * Modifies the entire side LED strips to be a single color.
         * @param color The color to apply.
         */
        private void set(Color color) {
            for (int i = 0; i < kStripLength; i++) {
                buffer.setRGB(i, color.r(), color.g(), color.b());
            }
        }

        /**
         * Modifies the buffer with values mirrored across the "center" of the LED strip.
         * @param i The index of the buffer to modify.
         * @param color The color to apply.
         */
        private void set(int i, Color color) {
            set(i, color.r(), color.g(), color.b());
        }

        /**
         * Modifies the buffer with values mirrored across the "center" of the LED strip.
         * @param i The index of the buffer to modify.
         * @param r Red value from {@code 0} to {@code 255}.
         * @param g Green value from {@code 0} to {@code 255}.
         * @param b Blue value from {@code 0} to {@code 255}.
         */
        private void set(int i, int r, int g, int b) {
            if (i >= kStripLength) return;
            buffer.setRGB(i, r, g, b);
            buffer.setRGB(buffer.getLength() - i - 1, r, g, b);
        }

        /**
         * Displays the currently selected reef level.
         * @param level The selected level (1-4). L1 is 1, not 0.
         */
        public Command levelSelection(Supplier<Integer> level) {
            return commandBuilder()
                .onExecute(() -> {
                    for (int i = 0; i < kStripLength; i++) {
                        if (i < ((28 / 4) * level.get())) {
                            set(i, Color.kLx);
                        } else {
                            set(i, Color.kOff);
                        }
                    }
                })
                .onEnd(() -> set(Color.kOff))
                .ignoringDisable(true)
                .withName("Lights.Sides.levelSelection(" + level + ")");
        }

        /**
         * Displays the flames animation.
         */
        public Command flames() {
            int[] state = new int[kStripLength];
            for (int i = 0; i < state.length; i++) {
                state[i] = 0;
            }

            return commandBuilder()
                .onExecute(() -> {
                    for (int i = 0; i < kStripLength; i++) {
                        state[i] = (int) Math.max(
                            0.0,
                            state[i] - (Math2.random((0.5 + (i / (kStripLength * 0.125))) * 5.0) + 28.0)
                        );
                    }
                    for (int i = kStripLength - 1; i >= 2; i--) {
                        state[i] = (state[i - 1] + state[i - 2] + state[i - 2]) / 3;
                    }
                    if (Math.random() < 0.5) {
                        int i = (int) Math2.random(5.0);
                        state[i] = (int) (state[i] + Math2.random(160.0, 255.0));
                    }
                    for (int i = 0; i < kStripLength; i++) {
                        int heat = (int) ((state[i] / 255.0) * 191.0);
                        int ramp = (heat & 63) << 2;
                        if (heat > 180) {
                            set(i, 255, 255, ramp);
                        } else if (heat > 60) {
                            set(i, 255, ramp, 0);
                        } else {
                            set(i, ramp, 0, 0);
                        }
                    }
                })
                .onEnd(() -> set(Color.kOff))
                .ignoringDisable(true)
                .withName("Lights.Sides.flames()");
        }

        /**
         * Turns the lights off.
         */
        public Command off() {
            return commandBuilder()
                .onInitialize(() -> set(Color.kOff))
                .ignoringDisable(true)
                .withName("Lights.Sides.off()");
        }
    }

    @Logged
    public class Top extends GRRSubsystem {

        private Top() {}

        /**
         * Modifies the entire side LED strips to be a single color.
         * @param color The color to apply.
         */
        private void set(Color color) {
            for (int i = kStripLength; i < kStripLength * 2; i++) {
                buffer.setRGB(i, color.r(), color.g(), color.b());
            }
        }

        /**
         * Displays the "Has Coral" animation.
         * @return
         */
        public Command hasCoral() {
            Timer timer = new Timer();

            return commandBuilder()
                .onInitialize(() -> timer.restart())
                .onExecute(() -> set(timer.get() % 0.4 > 0.2 ? Color.kHasCoral : Color.kOff))
                .onEnd(() -> set(Color.kOff))
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
                .onExecute(() -> set(timer.get() % 0.2 > 0.1 ? Color.kScored : Color.kOff))
                .onEnd(() -> set(Color.kOff))
                .withTimeout(1.5)
                .ignoringDisable(true)
                .withName("Lights.Top.scored()");
        }

        /**
         * Turns the lights off.
         */
        public Command off() {
            return commandBuilder()
                .onInitialize(() -> set(Color.kOff))
                .ignoringDisable(true)
                .withName("Lights.Top.off()");
        }
    }
}
