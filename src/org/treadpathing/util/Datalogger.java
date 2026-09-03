package org.treadpathing.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

/**
 * Buffered CSV logging, written where OnBotJava can hand it back to you.
 *
 * <p>Files land in {@code /sdcard/FIRST/java/src/Datalogs}, which is inside the OnBotJava
 * source tree — so a log appears in the editor's file browser with a one-click download, no
 * adb and no cable. That is the whole reason this class exists: FTC Dashboard cannot work
 * from OnBotJava (it serves its UI from Android assets, and OnBotJava cannot upload an AAR
 * with assets), so there is no live plotting. Logging a run and reading it back in the
 * visualizer replaces it, and gives you a record you can keep between matches.
 *
 * <p>The extension is {@code .txt} rather than {@code .csv} because the OnBotJava file browser
 * will not offer to download an unknown extension.
 *
 * <p>Writes are buffered and flushed on {@link #close()}. Call close in your OpMode's
 * {@code stop()} or a {@code finally} block; a log that is never closed loses its tail.
 */
public final class Datalogger {

    public static final String DEFAULT_DIRECTORY = "/sdcard/FIRST/java/src/Datalogs";

    private final Writer writer;
    private final int columns;
    private final StringBuilder line = new StringBuilder(256);
    private boolean closed;
    private int pending;

    /**
     * @param filename base name, without extension
     * @param headers  column names; the first column is always elapsed seconds
     */
    public Datalogger(String filename, String[] headers) {
        this(DEFAULT_DIRECTORY, filename, headers);
    }

    public Datalogger(String directory, String filename, String[] headers) {
        this.columns = headers.length;
        Writer opened = null;
        try {
            File folder = new File(directory);
            if (!folder.exists()) {
                folder.mkdirs();
            }
            opened = new BufferedWriter(new FileWriter(new File(folder, filename + ".txt")), 8192);
            opened.write("seconds");
            for (int i = 0; i < headers.length; i++) {
                opened.write(",");
                opened.write(headers[i]);
            }
            opened.write("\n");
        } catch (IOException failure) {
            // A missing SD card or a permissions problem must never take an auto down with it.
            opened = null;
        }
        this.writer = opened;
    }

    public boolean isOpen() {
        return writer != null && !closed;
    }

    /** Writes one row. Extra values past the declared column count are ignored. */
    public void write(double seconds, double... values) {
        if (!isOpen()) {
            return;
        }
        line.setLength(0);
        line.append(fixed(seconds));
        int count = Math.min(values.length, columns);
        for (int i = 0; i < count; i++) {
            line.append(',').append(fixed(values[i]));
        }
        for (int i = count; i < columns; i++) {
            line.append(',');
        }
        line.append('\n');
        try {
            writer.write(line.toString());
            pending++;
            if (pending >= 50) {
                writer.flush();
                pending = 0;
            }
        } catch (IOException failure) {
            closeQuietly();
        }
    }

    public void close() {
        closeQuietly();
    }

    private void closeQuietly() {
        if (writer == null || closed) {
            return;
        }
        closed = true;
        try {
            writer.flush();
            writer.close();
        } catch (IOException ignored) {
            // Nothing useful to do on the robot.
        }
    }

    /**
     * Four decimal places without {@code String.format}, which allocates a formatter per call
     * and is measurable at 50 Hz.
     */
    private static String fixed(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return "";
        }
        long scaled = Math.round(value * 10000.0);
        boolean negative = scaled < 0;
        if (negative) {
            scaled = -scaled;
        }
        long whole = scaled / 10000L;
        long fraction = scaled % 10000L;
        StringBuilder out = new StringBuilder(16);
        if (negative) {
            out.append('-');
        }
        out.append(whole).append('.');
        if (fraction < 1000) out.append('0');
        if (fraction < 100) out.append('0');
        if (fraction < 10) out.append('0');
        out.append(fraction);
        return out.toString();
    }
}
