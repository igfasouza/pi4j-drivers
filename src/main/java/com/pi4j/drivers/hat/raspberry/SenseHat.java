package com.pi4j.drivers.hat.raspberry;

import com.pi4j.context.Context;
import com.pi4j.drivers.display.BitmapFont;
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

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class SenseHat {

    private static final int WIDTH = 8;
    private static final int HEIGHT = 8;

    private static final int[] DEFAULT_GAMMA = createDefaultGamma();

    private static final BitmapFont MESSAGE_FONT = BitmapFont.get5x8Font(BitmapFont.Option.PROPORTIONAL);
    private static final BitmapFont LETTER_FONT = BitmapFont.get5x8Font();

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

    private GraphicsDisplay.Rotation rotation = GraphicsDisplay.Rotation.ROTATE_0;
    private boolean lowLight = false;
    private int[] gamma = DEFAULT_GAMMA.clone();

    private final int[][][] pixelBuffer = new int[HEIGHT][WIDTH][3];
    private final BlockingQueue<JoystickEvent> eventQueue = new LinkedBlockingQueue<>();

    public enum JoystickDirection {
        UP, DOWN, LEFT, RIGHT, MIDDLE
    }

    public enum JoystickAction {
        PRESSED, RELEASED, HELD
    }

    public record JoystickEvent(JoystickDirection direction, JoystickAction action, long timestamp) {
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
            LinuxInputDriver driver = getInputDriver();
            driver.addListener(this::handleEvent);

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
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j)
                    .bus(1)
                    .device(Hts221Driver.I2C_ADDRESS));
            hts221Driver = new Hts221Driver(i2c);
        }

        return hts221Driver;
    }

    public Lps25hDriver getPressureSensor() {
        if (lps25hDriver == null) {
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j)
                    .bus(1)
                    .device(Lps25hDriver.I2C_ADDRESS));
            lps25hDriver = new Lps25hDriver(i2c);
        }

        return lps25hDriver;
    }

    public Tcs3400Driver getLightSensor() {
        if (tcs3400Driver == null) {
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j)
                    .bus(1)
                    .device(Tcs3400Driver.I2C_ADDRESS));
            tcs3400Driver = new Tcs3400Driver(i2c);
        }

        return tcs3400Driver;
    }

    public Lsm9ds1Driver getImuSensor() {
        if (lsm9ds1Driver == null) {
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j)
                    .bus(1)
                    .device(Lsm9ds1Driver.I2C_ADDRESS_0));
            lsm9ds1Driver = new Lsm9ds1Driver(i2c);
        }

        return lsm9ds1Driver;
    }

    public Lsm9ds1MagnetometerDriver getMagnetometerSensor() {
        if (lsm9ds1MagnetometerDriver == null) {
            I2C i2c = pi4j.create(I2C.newConfigBuilder(pi4j)
                    .bus(1)
                    .device(Lsm9ds1MagnetometerDriver.I2C_ADDRESS_0));
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
            display = new GraphicsDisplay(getDisplayDriver(), rotation);
        }

        return display;
    }

    public void setPixel(int x, int y, int r, int g, int b) {
        validatePixelCoordinate(x, y);
        validateRgb(r, g, b);

        setPixelInBuffer(x, y, r, g, b);
        renderPixel(x, y);
        getDisplay().flush();
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
        validatePixelArray(pixels);

        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                setPixelInBuffer(x, y, pixels[y][x][0], pixels[y][x][1], pixels[y][x][2]);
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
        validateRgb(r, g, b);

        fillLogicalBuffer(r, g, b);
        renderBuffer();
    }

    public void showLetter(char letter) {
        showLetter(letter, new int[] { 255, 255, 255 }, new int[] { 0, 0, 0 });
    }

    public void showLetter(char letter, int[] textColor, int[] backColor) {
        validateColor(textColor);
        validateColor(backColor);

        fillLogicalBuffer(backColor[0], backColor[1], backColor[2]);

        BitmapFont.Glyph glyph = getGlyphOrSpace(LETTER_FONT, letter);
        int xOffset = Math.max(0, (WIDTH - glyph.getWidth()) / 2);
        int yOffset = Math.max(0, (HEIGHT - LETTER_FONT.getCellHeight()) / 2);

        drawGlyphToLogicalBuffer(glyph, LETTER_FONT, xOffset, yOffset, textColor);
        renderBuffer();
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

        int[][][] messageBuffer = buildTextBuffer(message, textColor, backColor);
        
        if(scrollSpeed <= 0) {
            throw new IllegalArgumentException("Scroll speed must be greater than zero");

        }
        int delayMillis = Math.max(1, (int) (scrollSpeed * 1000));

        for (int offset = 0; offset <= messageBuffer[0].length - WIDTH; offset++) {
            copyWindowToLogicalBuffer(messageBuffer, offset);
            renderBuffer();
            sleep(delayMillis);
        }
    }

    public void setRotation(int degrees) {
        this.rotation = switch (degrees) {
            case 0 -> GraphicsDisplay.Rotation.ROTATE_0;
            case 90 -> GraphicsDisplay.Rotation.ROTATE_90;
            case 180 -> GraphicsDisplay.Rotation.ROTATE_180;
            case 270 -> GraphicsDisplay.Rotation.ROTATE_270;
            default -> throw new IllegalArgumentException("Rotation must be 0, 90, 180 or 270 degrees");
        };

        this.display = new GraphicsDisplay(getDisplayDriver(), rotation);
        renderBuffer();
    }

    public int getRotation() {
        return switch (rotation) {
            case ROTATE_0 -> 0;
            case ROTATE_90 -> 90;
            case ROTATE_180 -> 180;
            case ROTATE_270 -> 270;
        };
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
        return getAccelerometerRaw();
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

    public int[] getColor() {
        float[] raw = getLightSensor().readCrgb();
        double clear = raw[0];

        if (clear <= 0) {
            return new int[] { 0, 0, 0 };
        }

        return new int[] {
                normalizeColor(raw[1], clear),
                normalizeColor(raw[2], clear),
                normalizeColor(raw[3], clear)
        };
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

        updateControllerState(event.getCode(), state);

        JoystickDirection direction = mapJoystickDirection(event.getCode());

        if (direction != null) {
            JoystickAction action = state ? JoystickAction.PRESSED : JoystickAction.RELEASED;
            eventQueue.offer(new JoystickEvent(direction, action, System.currentTimeMillis()));
        }
    }

    private void updateControllerState(int code, boolean state) {
        switch (code) {
            case LinuxInputDriver.KEY_DOWN -> down.setState(state);
            case LinuxInputDriver.KEY_UP -> up.setState(state);
            case LinuxInputDriver.KEY_LEFT -> left.setState(state);
            case LinuxInputDriver.KEY_RIGHT -> right.setState(state);
            case LinuxInputDriver.KEY_ENTER -> center.setState(state);
            default -> {
            }
        }
    }

    private JoystickDirection mapJoystickDirection(int code) {
        return switch (code) {
            case LinuxInputDriver.KEY_DOWN -> JoystickDirection.DOWN;
            case LinuxInputDriver.KEY_UP -> JoystickDirection.UP;
            case LinuxInputDriver.KEY_LEFT -> JoystickDirection.LEFT;
            case LinuxInputDriver.KEY_RIGHT -> JoystickDirection.RIGHT;
            case LinuxInputDriver.KEY_ENTER -> JoystickDirection.MIDDLE;
            default -> null;
        };
    }

    private void renderBuffer() {
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                renderPixel(x, y);
            }
        }

        getDisplay().flush();
    }

    private void renderPixel(int x, int y) {
        getDisplay().setPixel(
                x,
                y,
                rgb(
                        applyBrightness(pixelBuffer[y][x][0]),
                        applyBrightness(pixelBuffer[y][x][1]),
                        applyBrightness(pixelBuffer[y][x][2])));
    }

    private int[][][] buildTextBuffer(String message, int[] textColor, int[] backColor) {
        int width = WIDTH + measureTextWidth(message) + WIDTH;
        int[][][] buffer = new int[HEIGHT][width][3];

        fillBuffer(buffer, backColor);

        int cursorX = WIDTH;

        for (int offset = 0; offset < message.length();) {
            int codePoint = message.codePointAt(offset);
            offset += Character.charCount(codePoint);

            BitmapFont.Glyph glyph = getGlyphOrSpace(MESSAGE_FONT, codePoint);

            for (int y = 0; y < Math.min(HEIGHT, MESSAGE_FONT.getCellHeight()); y++) {
                for (int x = 0; x < glyph.getWidth(); x++) {
                    if (glyph.getPixel(x, y)) {
                        buffer[y][cursorX + x][0] = textColor[0];
                        buffer[y][cursorX + x][1] = textColor[1];
                        buffer[y][cursorX + x][2] = textColor[2];
                    }
                }
            }

            cursorX += glyph.getWidth();
        }

        return buffer;
    }

    private int measureTextWidth(String message) {
        int width = 0;

        for (int offset = 0; offset < message.length();) {
            int codePoint = message.codePointAt(offset);
            offset += Character.charCount(codePoint);

            width += getGlyphOrSpace(MESSAGE_FONT, codePoint).getWidth();
        }

        return width;
    }

    private BitmapFont.Glyph getGlyphOrSpace(BitmapFont font, int codePoint) {
        BitmapFont.Glyph glyph = font.getGlyph(codePoint);

        if (glyph != null) {
            return glyph;
        }

        glyph = font.getGlyph(' ');

        if (glyph != null) {
            return glyph;
        }

        throw new IllegalStateException("BitmapFont does not contain a space glyph");
    }

    private void drawGlyphToLogicalBuffer(BitmapFont.Glyph glyph, BitmapFont font, int xOffset, int yOffset,
            int[] color) {
        for (int y = 0; y < font.getCellHeight(); y++) {
            for (int x = 0; x < glyph.getWidth(); x++) {
                int targetX = x + xOffset;
                int targetY = y + yOffset;

                if (targetX >= 0
                        && targetX < WIDTH
                        && targetY >= 0
                        && targetY < HEIGHT
                        && glyph.getPixel(x, y)) {

                    setPixelInBuffer(targetX, targetY, color[0], color[1], color[2]);
                }
            }
        }
    }

    private void copyWindowToLogicalBuffer(int[][][] source, int offset) {
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                setPixelInBuffer(
                        x,
                        y,
                        source[y][offset + x][0],
                        source[y][offset + x][1],
                        source[y][offset + x][2]);
            }
        }
    }

    private void fillLogicalBuffer(int r, int g, int b) {
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                setPixelInBuffer(x, y, r, g, b);
            }
        }
    }

    private void fillBuffer(int[][][] buffer, int[] color) {
        for (int y = 0; y < buffer.length; y++) {
            for (int x = 0; x < buffer[y].length; x++) {
                buffer[y][x][0] = color[0];
                buffer[y][x][1] = color[1];
                buffer[y][x][2] = color[2];
            }
        }
    }

    private void setPixelInBuffer(int x, int y, int r, int g, int b) {
        pixelBuffer[y][x][0] = r;
        pixelBuffer[y][x][1] = g;
        pixelBuffer[y][x][2] = b;
    }

    private void swapPixels(int x1, int y1, int x2, int y2) {
        int[] temp = pixelBuffer[y1][x1];
        pixelBuffer[y1][x1] = pixelBuffer[y2][x2];
        pixelBuffer[y2][x2] = temp;
    }

    private int applyBrightness(int value) {
        int adjusted = lowLight ? Math.max(0, value / 3) : value;
        return gamma[adjusted];
    }

    private static int[] createDefaultGamma() {
        int[] values = new int[256];

        for (int i = 0; i < values.length; i++) {
            values[i] = i;
        }

        return values;
    }

    private static int normalizeColor(double value, double clear) {
        int normalized = (int) Math.round((value / clear) * 255.0);
        return Math.max(0, Math.min(255, normalized));
    }

    private static int rgb(int r, int g, int b) {
        return 0xff000000 | (r << 16) | (g << 8) | b;
    }

    private static void validatePixelCoordinate(int x, int y) {
        if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
            throw new IllegalArgumentException("Pixel coordinates must be between 0 and 7");
        }
    }

    private static void validatePixelArray(int[][][] pixels) {
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
            }
        }
    }

    private static void validateColor(int[] color) {
        if (color == null || color.length != 3) {
            throw new IllegalArgumentException("Color must be an int[3] RGB array");
        }

        validateRgb(color[0], color[1], color[2]);
    }

    private static void validateRgb(int r, int g, int b) {
        validateColorValue(r);
        validateColorValue(g);
        validateColorValue(b);
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
}