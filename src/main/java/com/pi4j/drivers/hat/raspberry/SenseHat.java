package com.pi4j.drivers.hat.raspberry;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pi4j.context.Context;
import com.pi4j.drivers.display.graphics.GraphicsDisplay;
import com.pi4j.drivers.display.graphics.GraphicsDisplayDriver;
import com.pi4j.drivers.display.graphics.framebuffer.FramebufferDriver;
import com.pi4j.drivers.input.GameController;
import com.pi4j.drivers.input.linux.LinuxInputDriver;
import com.pi4j.drivers.sensor.Sensor;
import com.pi4j.drivers.sensor.environment.hts221.Hts221Driver;
import com.pi4j.drivers.sensor.environment.lps25h.Lps25hDriver;
import com.pi4j.drivers.sensor.environment.tcs3400.Tcs3400Driver;
import com.pi4j.drivers.sensor.geospatial.lsm9ds1.Lsm9ds1Driver;
import com.pi4j.drivers.sensor.geospatial.lsm9ds1.Lsm9ds1MagnetometerDriver;
import com.pi4j.io.ListenableOnOffRead;
import com.pi4j.io.i2c.I2C;

public class SenseHat {
    private final Context pi4j;
    private GameController controller;
    private GraphicsDisplayDriver displayDriver;
    private GraphicsDisplay display;
    private LinuxInputDriver inputDriver;
    private Hts221Driver hts221Driver;
    private Lps25hDriver lps25hDriver;
    private Tcs3400Driver tcs3400Driver;
    private Lsm9ds1Driver lsm9ds1Driver;
    private Lsm9ds1MagnetometerDriver lsm9ds1MagnetometerDriver;

    private final ListenableOnOffRead.Impl up = new ListenableOnOffRead.Impl();
    private final ListenableOnOffRead.Impl down = new ListenableOnOffRead.Impl();
    private final ListenableOnOffRead.Impl left = new ListenableOnOffRead.Impl();
    private final ListenableOnOffRead.Impl right = new ListenableOnOffRead.Impl();
    private final ListenableOnOffRead.Impl center = new ListenableOnOffRead.Impl();

    private static final int WIDTH = 8;
    private static final int HEIGHT = 8;

    private int rotation = 0;
    private boolean lowLight = false;

    private final int[][][] pixelBuffer = new int[HEIGHT][WIDTH][3];

    private static final int[] DEFAULT_GAMMA = createDefaultGamma();

    private int[] gamma = DEFAULT_GAMMA.clone();

    private final java.util.concurrent.BlockingQueue<JoystickEvent> eventQueue = new java.util.concurrent.LinkedBlockingQueue<>();

    public enum JoystickDirection {
        UP, DOWN, LEFT, RIGHT, MIDDLE
    }

    public enum JoystickAction {
        PRESSED, RELEASED, HELD
    }

    public record JoystickEvent(
            JoystickDirection direction,
            JoystickAction action,
            long timestamp) {
    }

    public SenseHat(Context pi4j) {
        this.pi4j = pi4j;
    }

    public LinuxInputDriver getInputDriver() {
        if (inputDriver == null) {
            inputDriver = LinuxInputDriver.forSenseHat();
        }
        return inputDriver;
    }

    public GameController getController() {
        if (controller == null) {
            LinuxInputDriver inputDriver = getInputDriver();
            inputDriver.addListener(this::handleEvent);
            controller = new GameController.Builder(pi4j)
                    .addDigitalInput(GameController.Key.DOWN, down)
                    .addDigitalInput(GameController.Key.LEFT, left)
                    .addDigitalInput(GameController.Key.RIGHT, right)
                    .addDigitalInput(GameController.Key.UP, up)
                    .addDigitalInput(GameController.Key.CENTER, center)
                    .build();
        }
        return controller;
    }

    public Hts221Driver getHumiditySensor() {
        if (hts221Driver == null) {
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j).bus(1).device(Hts221Driver.I2C_ADDRESS));
            hts221Driver = new Hts221Driver(i2c);
        }
        return hts221Driver;
    }

    public Lps25hDriver getPressureSensor() {
        if (lps25hDriver == null) {
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j).bus(1).device(Lps25hDriver.I2C_ADDRESS));
            lps25hDriver = new Lps25hDriver(i2c);
        }
        return lps25hDriver;
    }

    public Tcs3400Driver getLightSensor() {
        if (tcs3400Driver == null) {
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j).bus(1).device(Tcs3400Driver.I2C_ADDRESS));
            tcs3400Driver = new Tcs3400Driver(i2c);
        }
        return tcs3400Driver;
    }

    public Lsm9ds1Driver getImuSensor() {
        if (lsm9ds1Driver == null) {
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j).bus(1).device(Lsm9ds1Driver.I2C_ADDRESS_0));
            lsm9ds1Driver = new Lsm9ds1Driver(i2c);
        }
        return lsm9ds1Driver;
    }

    public Lsm9ds1MagnetometerDriver getMagnetometerSensor() {
        if (lsm9ds1MagnetometerDriver == null) {
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j).bus(1).device(Lsm9ds1MagnetometerDriver.I2C_ADDRESS_0));
            lsm9ds1MagnetometerDriver = new Lsm9ds1MagnetometerDriver(i2c);
        }
        return lsm9ds1MagnetometerDriver;
    }

    public List<Sensor> getAllSensors() {
        List<Sensor> result = new ArrayList<>();
        result.add(getHumiditySensor());
        result.add(getPressureSensor());
        result.add(getLightSensor());
        result.add(getImuSensor());
        result.add(getMagnetometerSensor());
        return result;
    }

    public GraphicsDisplayDriver getDisplayDriver() {
        if (displayDriver == null) {
            displayDriver = FramebufferDriver.forSenseHat();
        }
        return displayDriver;
    }

    public GraphicsDisplay getDisplay() {
        if (display == null) {
            display = new GraphicsDisplay(getDisplayDriver());
        }
        return display;
    }

    // ---------------------------------------------------------------------
    // Display
    // ---------------------------------------------------------------------

    public void setPixel(int x, int y, int r, int g, int b) {
        validatePixelCoordinate(x, y);
        validateColorValue(r);
        validateColorValue(g);
        validateColorValue(b);

        int[] mapped = mapCoordinates(x, y);
        int rr = applyLowLight(r);
        int gg = applyLowLight(g);
        int bb = applyLowLight(b);

        pixelBuffer[y][x][0] = r;
        pixelBuffer[y][x][1] = g;
        pixelBuffer[y][x][2] = b;

        getDisplay().setPixel(mapped[0], mapped[1], rgb(rr, gg, bb));
        getDisplay().flush();
    }

    private static int rgb(int r, int g, int b) {
        return 0xff000000 | (r << 16) | (g << 8) | b;
    }

    public int[] getPixel(int x, int y) {
        validatePixelCoordinate(x, y);

        return new int[] {
                pixelBuffer[y][x][0],
                pixelBuffer[y][x][1],
                pixelBuffer[y][x][2]
        };
    }

    public void setPixels(int[][][] pixels) {
        if (pixels == null || pixels.length != HEIGHT) {
            throw new IllegalArgumentException("Pixels must be an [8][8][3] array");
        }

        for (int y = 0; y < HEIGHT; y++) {
            if (pixels[y] == null || pixels[y].length != WIDTH) {
                throw new IllegalArgumentException("Pixels must be an [8][8][3] array");
            }

            for (int x = 0; x < WIDTH; x++) {
                if (pixels[y][x] == null || pixels[y][x].length != 3) {
                    throw new IllegalArgumentException("Pixels must be an [8][8][3] array");
                }

                validateColor(pixels[y][x]);
                pixelBuffer[y][x][0] = pixels[y][x][0];
                pixelBuffer[y][x][1] = pixels[y][x][1];
                pixelBuffer[y][x][2] = pixels[y][x][2];
            }
        }

        renderBuffer();
    }

    public int[][][] getPixels() {
        int[][][] copy = new int[HEIGHT][WIDTH][3];

        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                copy[y][x][0] = pixelBuffer[y][x][0];
                copy[y][x][1] = pixelBuffer[y][x][1];
                copy[y][x][2] = pixelBuffer[y][x][2];
            }
        }

        return copy;
    }

    public void clear() {
        clear(0, 0, 0);
    }

    public void clear(int r, int g, int b) {
        validateColorValue(r);
        validateColorValue(g);
        validateColorValue(b);

        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                pixelBuffer[y][x][0] = r;
                pixelBuffer[y][x][1] = g;
                pixelBuffer[y][x][2] = b;
            }
        }

        renderBuffer();
    }

    public void showLetter(char letter) {
        showLetter(letter, new int[] { 255, 255, 255 }, new int[] { 0, 0, 0 });
    }

    public void showLetter(char letter, int[] textColor, int[] backColor) {
        validateColor(textColor);
        validateColor(backColor);

        boolean[][] glyph = FONT.getOrDefault(
                Character.toUpperCase(letter),
                FONT.get(' '));

        clear(backColor[0], backColor[1], backColor[2]);

        for (int y = 0; y < 7; y++) {
            for (int x = 0; x < 5; x++) {
                if (glyph[y][x]) {
                    setPixel(x + 1, y, textColor[0], textColor[1], textColor[2]);
                }
            }
        }
    }

    public void showMessage(String message) {
        showMessage(message, 0.1, new int[] { 255, 255, 255 }, new int[] { 0, 0, 0 });
    }

    public void showMessage(String message, double scrollSpeed) {
        showMessage(message, scrollSpeed, new int[] { 255, 255, 255 }, new int[] { 0, 0, 0 });
    }

    public void showMessage(String message, double scrollSpeed, int[] textColor, int[] backColor) {
        if (message == null) {
            throw new IllegalArgumentException("Message cannot be null");
        }

        validateColor(textColor);
        validateColor(backColor);

        boolean[][] buffer = buildMessageBuffer(message.toUpperCase());
        int delayMillis = Math.max(1, (int) (scrollSpeed * 1000));

        for (int offset = 0; offset <= buffer[0].length - WIDTH; offset++) {
            drawWindow(buffer, offset, textColor, backColor);
            sleep(delayMillis);
        }
    }

    public void setRotation(int degrees) {
        if (degrees != 0 && degrees != 90 && degrees != 180 && degrees != 270) {
            throw new IllegalArgumentException("Rotation must be one of 0, 90, 180, 270");
        }

        this.rotation = degrees;
        renderBuffer();
    }

    public int getRotation() {
        return rotation;
    }

    public void flipH() {
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH / 2; x++) {
                swapPixels(x, y, WIDTH - 1 - x, y);
            }
        }

        renderBuffer();
    }

    public void flipV() {
        for (int y = 0; y < HEIGHT / 2; y++) {
            for (int x = 0; x < WIDTH; x++) {
                swapPixels(x, y, x, HEIGHT - 1 - y);
            }
        }

        renderBuffer();
    }

    public void setLowLight(boolean enabled) {
        this.lowLight = enabled;
        renderBuffer();
    }

    public boolean isLowLight() {
        return lowLight;
    }

    public void setGamma(int[] gamma) {
        if (gamma == null || gamma.length != 256) {
            throw new IllegalArgumentException("Gamma must be an int[256] array");
        }

        for (int value : gamma) {
            validateColorValue(value);
        }

        this.gamma = gamma.clone();
        renderBuffer();
    }

    public int[] getGamma() {
        return gamma.clone();
    }

    public void gammaReset() {
        this.gamma = DEFAULT_GAMMA.clone();
        renderBuffer();
    }

    private static int[] createDefaultGamma() {
        int[] values = new int[256];

        for (int i = 0; i < values.length; i++) {
            values[i] = i;
        }

        return values;
    }

    // ---------------------------------------------------------------------
    // Environment
    // ---------------------------------------------------------------------

    public double getHumidity() {
        return getHumiditySensor().readHumidity();
    }

    public double getPressure() {
        return getPressureSensor().readPressure();
    }

    public double getTemperature() {
        return getTemperatureFromHumidity();
    }

    public double getTemperatureFromHumidity() {
        return getHumiditySensor().readTemperature();
    }

    public double getTemperatureFromPressure() {
        return getPressureSensor().readTemperature();
    }

    // ---------------------------------------------------------------------
    // IMU - wire these to LSM9DS1 once method names are confirmed
    // ---------------------------------------------------------------------

    public double[] getOrientation() {
        return getOrientationDegrees();
    }

    public double[] getOrientationRadians() {
        double[] degrees = getOrientationDegrees();

        return new double[] {
                Math.toRadians(degrees[0]),
                Math.toRadians(degrees[1]),
                Math.toRadians(degrees[2])
        };
    }

    public double[] getOrientationDegrees() {
        double[] accel = getAccelerometerRaw();

        double x = accel[0];
        double y = accel[1];
        double z = accel[2];

        double pitch = Math.toDegrees(Math.atan2(x, Math.sqrt(y * y + z * z)));
        double roll = Math.toDegrees(Math.atan2(y, Math.sqrt(x * x + z * z)));
        double yaw = getCompassHeading();

        return new double[] { pitch, roll, yaw };
    }

    public double[] getAccelerometer() {
        return getOrientationDegrees();
    }

    public double[] getAccelerometerRaw() {
        float[] values = getImuSensor().readAccelerometer();

        return new double[] {
                values[0],
                values[1],
                values[2]
        };
    }

    public double[] getGyroscope() {
        return getGyroscopeRaw();
    }

    public double[] getGyroscopeRaw() {
        float[] values = getImuSensor().readGyroscope();

        return new double[] {
                values[0],
                values[1],
                values[2]
        };
    }

    public double[] getCompass() {
        return getCompassRaw();
    }

    public double[] getCompassRaw() {
        float[] values = getMagnetometerSensor().readMagneticField();

        return new double[] {
                values[0],
                values[1],
                values[2]
        };
    }

    public double getCompassHeading() {
        double[] mag = getCompassRaw();

        double heading = Math.toDegrees(Math.atan2(mag[1], mag[0]));

        if (heading < 0) {
            heading += 360.0;
        }

        return heading;
    }

    // ---------------------------------------------------------------------
    // Joystick
    // ---------------------------------------------------------------------

    public List<JoystickEvent> getEvents() {
        getController();

        List<JoystickEvent> events = new ArrayList<>();
        eventQueue.drainTo(events);

        return events;
    }

    public JoystickEvent waitForEvent() {
        getController();

        try {
            return eventQueue.take();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException("Interrupted while waiting for joystick event", e);
        }
    }

    private void handleEvent(LinuxInputDriver.Event event) {
        if (event.getType() != LinuxInputDriver.EV_KEY) {
            return;
        }
        Boolean state = switch (event.getValue()) {
            case LinuxInputDriver.STATE_PRESS -> true;
            case LinuxInputDriver.STATE_RELEASE -> false;
            default -> null;
        };
        if (state == null) {
            return;
        }
        JoystickAction action = state ? JoystickAction.PRESSED : JoystickAction.RELEASED;
        JoystickDirection direction = switch (event.getCode()) {
            case LinuxInputDriver.KEY_DOWN -> JoystickDirection.DOWN;
            case LinuxInputDriver.KEY_UP -> JoystickDirection.UP;
            case LinuxInputDriver.KEY_LEFT -> JoystickDirection.LEFT;
            case LinuxInputDriver.KEY_RIGHT -> JoystickDirection.RIGHT;
            case LinuxInputDriver.KEY_ENTER -> JoystickDirection.MIDDLE;
            default -> null;
        };

        if (direction != null) {
            eventQueue.offer(new JoystickEvent(direction, action, System.currentTimeMillis()));
        }

    }

    // ---------------------------------------------------------------------
    // Colour sensor
    // ---------------------------------------------------------------------

    public int[] getColor() {
        float[] raw = getLightSensor().readCrgb();

        double clear = raw[0];

        if (clear <= 0) {
            return new int[] { 0, 0, 0 };
        }

        int r = normalizeColor(raw[1], clear);
        int g = normalizeColor(raw[2], clear);
        int b = normalizeColor(raw[3], clear);

        return new int[] { r, g, b };
    }

    public int[] getColorRaw() {
        float[] raw = getLightSensor().readCrgb();

        return new int[] {
                Math.round(raw[1]),
                Math.round(raw[2]),
                Math.round(raw[3]),
                Math.round(raw[0])
        };
    }

    private static int normalizeColor(double value, double clear) {
        int normalized = (int) Math.round((value / clear) * 255.0);
        return Math.max(0, Math.min(255, normalized));
    }

    public int getColorGain() {
        throw new UnsupportedOperationException(
                "Tcs3400Driver does not currently expose gain. Add gain support to Tcs3400Driver first.");
    }

    public void setColorGain(int gain) {
        throw new UnsupportedOperationException(
                "Tcs3400Driver does not currently expose gain. Add gain support to Tcs3400Driver first.");
    }

    public int getIntegrationCycles() {
        throw new UnsupportedOperationException(
                "Tcs3400Driver does not currently expose integration cycles. Add integration support to Tcs3400Driver first.");
    }

    public void setIntegrationCycles(int cycles) {
        throw new UnsupportedOperationException(
                "Tcs3400Driver does not currently expose integration cycles. Add integration support to Tcs3400Driver first.");
    }

    // ---------------------------------------------------------------------
    // Internal helpers
    // ---------------------------------------------------------------------

    private void renderBuffer() {
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                int[] mapped = mapCoordinates(x, y);

                getDisplay().setPixel(
                        mapped[0],
                        mapped[1],
                        rgb(
                                applyLowLight(pixelBuffer[y][x][0]),
                                applyLowLight(pixelBuffer[y][x][1]),
                                applyLowLight(pixelBuffer[y][x][2])));
            }
        }

        getDisplay().flush();
    }

    private int[] mapCoordinates(int x, int y) {
        return switch (rotation) {
            case 90 -> new int[] { HEIGHT - 1 - y, x };
            case 180 -> new int[] { WIDTH - 1 - x, HEIGHT - 1 - y };
            case 270 -> new int[] { y, WIDTH - 1 - x };
            default -> new int[] { x, y };
        };
    }

    private void swapPixels(int x1, int y1, int x2, int y2) {
        int[] temp = pixelBuffer[y1][x1];

        pixelBuffer[y1][x1] = pixelBuffer[y2][x2];
        pixelBuffer[y2][x2] = temp;
    }

    private int applyLowLight(int value) {
        return lowLight ? Math.max(0, value / 3) : value;
    }

    private boolean[][] buildMessageBuffer(String message) {
        int charWidth = 5;
        int charHeight = 7;
        int spacing = 1;

        int width = WIDTH + message.length() * (charWidth + spacing) + WIDTH;
        boolean[][] buffer = new boolean[HEIGHT][width];

        int cursorX = WIDTH;

        for (char c : message.toCharArray()) {
            boolean[][] glyph = FONT.getOrDefault(c, FONT.get(' '));

            for (int y = 0; y < charHeight; y++) {
                for (int x = 0; x < charWidth; x++) {
                    buffer[y][cursorX + x] = glyph[y][x];
                }
            }

            cursorX += charWidth + spacing;
        }

        return buffer;
    }

    private void drawWindow(boolean[][] buffer, int offset, int[] textColor, int[] backColor) {
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                boolean on = buffer[y][offset + x];
                int[] color = on ? textColor : backColor;

                pixelBuffer[y][x][0] = color[0];
                pixelBuffer[y][x][1] = color[1];
                pixelBuffer[y][x][2] = color[2];
            }
        }

        renderBuffer();
    }

    private static void validatePixelCoordinate(int x, int y) {
        if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
            throw new IllegalArgumentException("Pixel coordinates must be between 0 and 7");
        }
    }

    private static void validateColor(int[] color) {
        if (color == null || color.length != 3) {
            throw new IllegalArgumentException("Color must be an int[3] RGB array");
        }

        validateColorValue(color[0]);
        validateColorValue(color[1]);
        validateColorValue(color[2]);
    }

    private static void validateColorValue(int value) {
        if (value < 0 || value > 255) {
            throw new IllegalArgumentException("Color values must be between 0 and 255");
        }
    }

    private static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private static boolean[][] glyph(String... rows) {
        boolean[][] result = new boolean[7][5];

        for (int y = 0; y < 7; y++) {
            for (int x = 0; x < 5; x++) {
                result[y][x] = rows[y].charAt(x) == '1';
            }
        }

        return result;
    }

    private static final Map<Character, boolean[][]> FONT = Map.ofEntries(
            Map.entry(' ', glyph("00000", "00000", "00000", "00000", "00000", "00000", "00000")),
            Map.entry('!', glyph("00100", "00100", "00100", "00100", "00100", "00000", "00100")),
            Map.entry('0', glyph("01110", "10001", "10011", "10101", "11001", "10001", "01110")),
            Map.entry('1', glyph("00100", "01100", "00100", "00100", "00100", "00100", "01110")),
            Map.entry('2', glyph("01110", "10001", "00001", "00010", "00100", "01000", "11111")),
            Map.entry('3', glyph("11110", "00001", "00001", "01110", "00001", "00001", "11110")),
            Map.entry('4', glyph("00010", "00110", "01010", "10010", "11111", "00010", "00010")),
            Map.entry('5', glyph("11111", "10000", "10000", "11110", "00001", "00001", "11110")),
            Map.entry('6', glyph("01110", "10000", "10000", "11110", "10001", "10001", "01110")),
            Map.entry('7', glyph("11111", "00001", "00010", "00100", "01000", "01000", "01000")),
            Map.entry('8', glyph("01110", "10001", "10001", "01110", "10001", "10001", "01110")),
            Map.entry('9', glyph("01110", "10001", "10001", "01111", "00001", "00001", "01110")),
            Map.entry('A', glyph("01110", "10001", "10001", "11111", "10001", "10001", "10001")),
            Map.entry('B', glyph("11110", "10001", "10001", "11110", "10001", "10001", "11110")),
            Map.entry('C', glyph("01110", "10001", "10000", "10000", "10000", "10001", "01110")),
            Map.entry('D', glyph("11110", "10001", "10001", "10001", "10001", "10001", "11110")),
            Map.entry('E', glyph("11111", "10000", "10000", "11110", "10000", "10000", "11111")),
            Map.entry('F', glyph("11111", "10000", "10000", "11110", "10000", "10000", "10000")),
            Map.entry('G', glyph("01110", "10001", "10000", "10111", "10001", "10001", "01110")),
            Map.entry('H', glyph("10001", "10001", "10001", "11111", "10001", "10001", "10001")),
            Map.entry('I', glyph("11111", "00100", "00100", "00100", "00100", "00100", "11111")),
            Map.entry('J', glyph("00111", "00010", "00010", "00010", "00010", "10010", "01100")),
            Map.entry('K', glyph("10001", "10010", "10100", "11000", "10100", "10010", "10001")),
            Map.entry('L', glyph("10000", "10000", "10000", "10000", "10000", "10000", "11111")),
            Map.entry('M', glyph("10001", "11011", "10101", "10101", "10001", "10001", "10001")),
            Map.entry('N', glyph("10001", "11001", "10101", "10011", "10001", "10001", "10001")),
            Map.entry('O', glyph("01110", "10001", "10001", "10001", "10001", "10001", "01110")),
            Map.entry('P', glyph("11110", "10001", "10001", "11110", "10000", "10000", "10000")),
            Map.entry('Q', glyph("01110", "10001", "10001", "10001", "10101", "10010", "01101")),
            Map.entry('R', glyph("11110", "10001", "10001", "11110", "10100", "10010", "10001")),
            Map.entry('S', glyph("01111", "10000", "10000", "01110", "00001", "00001", "11110")),
            Map.entry('T', glyph("11111", "00100", "00100", "00100", "00100", "00100", "00100")),
            Map.entry('U', glyph("10001", "10001", "10001", "10001", "10001", "10001", "01110")),
            Map.entry('V', glyph("10001", "10001", "10001", "10001", "10001", "01010", "00100")),
            Map.entry('W', glyph("10001", "10001", "10001", "10101", "10101", "10101", "01010")),
            Map.entry('X', glyph("10001", "10001", "01010", "00100", "01010", "10001", "10001")),
            Map.entry('Y', glyph("10001", "10001", "01010", "00100", "00100", "00100", "00100")),
            Map.entry('Z', glyph("11111", "00001", "00010", "00100", "01000", "10000", "11111")));

}