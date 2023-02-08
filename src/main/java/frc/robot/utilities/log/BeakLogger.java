// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

/** Generic data logger, with support for filtering.. */
public class BeakLogger {
    private SimpleDateFormat m_formatter;
    private PrintWriter m_writer;

    private String m_logPath;

    /**
     * Create a new logger
     * @param parentFolder The parent folder to write this to, e.g. <code>/media/sda1</code> for a USB stick
     * @param mode The "mode" to append to the filename, e.g. auton/teleop
     * @throws IOException If the folder can't be written to.
     */
    public BeakLogger(String parentFolder, String mode) throws IOException {
        m_formatter = new SimpleDateFormat("yyyy-MM-dd_HH_mm_ss.SSS");
        m_formatter.setTimeZone(TimeZone.getTimeZone("US/Eastern"));

        String logFilename = m_formatter.format(new Date()) + "_" + mode + ".tsv";

        m_logPath = parentFolder + File.separator + logFilename;

        m_writer = new PrintWriter(new BufferedWriter(new FileWriter(m_logPath, true)));
    }
    
    /**
     * Log generic data.
     * @param data Data to print.
     */
    public void logData(String data) {
        m_writer.print("[" + m_formatter.format(new Date()) + "]" + data + "\n");
        m_writer.flush();
    }

    /**
     * Log severe warnings, errors, etc.
     * @param data Data to print.
     */
    public void logSevere(String data) {
        logData("SEVERE: " + data);
    }

    /**
     * Log generic warnings and errors.
     * @param data Data to print.
     */
    public void logWarning(String data) {
        logData("WARNING: " + data);
    }

    /**
     * Log generic info.
     * @param data Data to print.
     */
    public void logInfo(String data) {
        logData("INFO: " + data);
    }

    /**
     * Log debug info.
     * @param data Data to print.
     */
    public void logDebug(String data) {
        logData("DEBUG: " + data);
    }

    /**
     * Log configuration info.
     * @param data Data to print.
     */
    public void logConfig(String data) {
        logData("CONFIG: " + data);
    }
}