package org.usfirst.frc2016;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/****************************************************************************************************
 * General purpose tool to allow user to record and save robot variable data to
 * .xls spreadsheet.
 *
 * <p>
 * 
 * <pre>
* Changes ********************************************************************************************
* Date          Ver.    BY      Description
* Dec. 26, 2015 1.03    FWL	    1) Added comments to discuss how to get more precision for numbers.
*                               2) Fixed the Tele_FileName so if Preferences value is zero length (empty) 
*                                  the name reverts to telemetry without having to delete the preferences
*                                  key Tele_FileName
* 
* Dec. 17, 2015 1.02    FWL     Added Java docs and changed a few things. 
*                               1) changed method addHeadedr to addColumn. 
*                               2) changed headerName to columnName.
*                                
* Dec. 16, 2015 1.01    FWL     Added commenting and changed names to relate it is a spread sheet.
*
* Dec. 14, 2015 1.00    FWL     Initial submission
 * </pre>
 * <p>
 * 
 * @author Frank Larkin, FRC272 - Lansdale Catholic Robotics pafwl@aol.com
 * @version 1.03
 * 
 */
public class LCTelemetry {

    private String[] listRows; // rows of saved data

    private int savedRows; // rows of data produced
    private Timer timerStart; // time from last rest
    private double timeLast; // time of last save
    private DriverStation driverStation;
    private boolean timeStampFilename; // true to add date stamp to filename
    private String filename;
    /** Default: telemetry.<b>Note: xls is added to the end.</b> */
    private String filePath;
    /**
     * Default: /tmp folder.(linked to /var/volitile/tmp) Lost after reboot,
     * /home/lvuser is writable.
     */

    public final int maxRows = 50 * 60 * 3;
    /** Default 9000: 50 times per second * 60 seconds * 3 minutes */

    private DateFormat dateFormat;

    // column headers for default data
    private String defaultColumnNames = "Timer,Scan MS,Date Time,Event,Match Type,Match Number,Batt Volts,Brown Out,Mode";

    // dictionary for the column names
    private Map<String, Integer> dictColumnNames;
    // storage for building current row data
    private String[] currentRowData;

    private final int scanHistSize = 100; // total timing buckets
    private int[] scanHist = new int[scanHistSize];

    /**
     * Class Constructor initialize you variables here.
     * Here the size of the arrays can be adjusted to allow for more columns
     * and more rows. The constants ki_MaxRos and ki_MaxColumns control that.
     */
    public LCTelemetry() {
        timerStart = new Timer();
        timerStart.start();

        driverStation = DriverStation.getInstance();
        timeStampFilename = true;
        filePath = "/var/volatile/tmp"; /** /var/volatile/tmp folder. Lost after reboot, /home/lvuser is writable. */
        filename = "telemetry";
        dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");

        // initialize dictionary with default columns
        dictColumnNames = new HashMap<String, Integer>();
        for (String columnName : defaultColumnNames.split(",")) {
            addColumn(columnName);
        }

        createNewList();

    }

    private void createNewList() {
        timerStart.reset();
        timeLast = 0;

        savedRows = 0;
        listRows = new String[maxRows];

        scanHist = new int[scanHistSize];
        currentRowData = new String[dictColumnNames.size()];
    }

    /**
     * Used to reset the Telemetry internal timer to zero. You have to add this
     * method to be called in every Init method of the main Iterative Robot class,
     * disabledInit, teleopInit, autonomousInit.<br>
     * This way you will be able to see the actual time spent in each section.
     * <p>
     * Example:
     * <p>
     * 
     * <pre>
     * public void teleopInit(){
     * 
     * telem.restartTimer();
     * 
     * }
     * 
     * <pre>
     */
    public void restartTimer() {
        createNewList();
    }

    /**
     * Called to add a column header to the Telemetry object. The field should
     * have spaces between words to allow them to easily word wrap in
     * Excel. The exact spelling is needed to add column data.<br>
     * Example: I Left Driver Power
     * <b>Note: The I tells us this is from the Inputs Class.</b>
     * <p>
     * The order these are entered in will be the columns of the
     * final spread sheet. The order the data is add has no impact on this.
     * <p>
     * It is assumed that you will add a method to each class like
     * addTelemetryHeaders(). In that class you will add the different fields
     * you want to track. This is best called in the constructor of the Robot
     * class after all the other classes have been initialized.
     * <p>
     * Example:
     * 
     * <pre>
     * public void addTelemetryHeaders(LCTelemetry telem ){
     * telem.addColumn("A Program Number");
     * telem.addColumn("A Started");
     * telem.addColumn("A Step Timer");
     * }
     * <p>
     * 
     * @param columnName string at the top of this column.
     */
    public void addColumn(String columnName) {
        if (!dictColumnNames.containsKey(columnName)) {
            // System.out.println("LCTelemetry: adding column " + columnName);
            dictColumnNames.put(columnName, dictColumnNames.size());
        }
    }

    /**
     * Called to add a string to a column in the
     * Telemetry instance. The columnName is the exact spelling
     * of the name entered in an addColumn call. The order this data
     * is added will not change the order of the columns. No additional
     * formatting is done on the data.
     * <p>
     * Example: See saveString()
     * <p>
     * If you have a double that you want to save to more precision
     * convert to a sting and save as a string. As in saving an
     * accelerometer value you may want 6 decimal places.
     * 
     * <pre>
     * telem.saveString("S ACCEL X", String.format("%.6f", this.d_BACC_XAxis));
     * </pre>
     * <p>
     * 
     * @param columnName string at the top of this column.
     * @param value      String to be written.
     */
    public void saveString(String columnName, String value) {
        int colIndex = dictColumnNames.getOrDefault(columnName, -1);
        if (colIndex >= 0 && colIndex < currentRowData.length) {
            currentRowData[colIndex] = value;
        }
    }

    /**
     * Called to add an Integer to a column in the Telemetry instance.
     * The columnName is the exact spelling of the name entered in an
     * addColumn call. The order this data is added will not change the
     * order of the columns.
     * <p>
     * Example:
     * 
     * <pre>
     * public void writeTelemetryValues(LCTelemetry telem) {
     *     telem.saveInteger("A Program Number", this.ip_ProgramNumber);
     *     telem.saveTrueBoolean("A Started", this.b_Started);
     *     telem.saveTrueBoolean("A Is Done", this.b_IsDone);
     *     telem.saveDouble("A Auton Timer", this.tim_AutonTimer.get());
     *     telem.saveString("A Program Desc", this.s_ProgramDescription);
     *     telem.saveString("A Step Desc", this.s_StepDescription);
     *     telem.saveInteger("A Step", this.i_Step);
     *     telem.saveDouble("A Step Timer", this.tim_StepTimer.get());
     * }
     * </pre>
     * <p>
     * 
     * @param columnName string at the top of this column.
     * @param value      Integer number to be written.
     */
    public void saveInteger(String columnName, int value) {
        saveString(columnName, String.format("%d", value));
    }

    /**
     * Called to add an Double float point number to a column in the
     * Telemetry instance. The columnName is the exact spelling
     * of the name entered in an addColumn call. The order this data
     * is added will not change the order of the columns. The resulting
     * data will have 2 decimal points of precision.
     * <p>
     * Default precision is .2f or 2 decimal places. To save with more
     * precision convert to string and save as a sting. See saveString.
     * 
     * 
     * @param columnName string at the top of this column.
     * @param value      Double number to be written.
     */
    public void saveDouble(String columnName, double value) {
        saveString(columnName, String.format("%.2f", value)); // format as floating point with 2 decimals.
    }

    /**
     * Called to add a true/false boolean to a column in the
     * Telemetry instance. The columnName is the exact spelling
     * of the name entered in an addColumn call. The order this data
     * is added will not change the order of the columns. No additional
     * formatting is done on the data but Excel will fore these to CAPs.
     * <p>
     * Example: See saveInteger()
     * <p>
     * 
     * @param columnName string at the top of this column.
     * @param value      Boolean to be written.
     */
    public void saveBoolean(String columnName, Boolean value) {
        saveString(columnName, value.toString()); // save the Boolean as a string
    }

    /**
     * Called to add a true boolean to a column in the Telemetry instance.
     * If the item is false then an empty field will be added. If true
     * then the field will say true. Excel will set this to all CAPS.
     * <p>
     * The columnName is the exact spelling of the name entered in an
     * addColumn call. The order this data is added will not change
     * the order of the columns.
     * <p>
     * Example: See saveInteger()
     * <p>
     * 
     * @param columnName string at the top of this column.
     * @param value      boolean to be written.
     */
    public void saveTrueBoolean(String columnName, Boolean value) { // we only want true or blank easier to read
        saveString(columnName, (value == true) ? value.toString() : "");
    }

    /**
     * Called to add a false boolean to a column in the Telemetry instance.
     * If the item is true then an empty field will be added. If false
     * then the field will say false. Excel will set this to all CAPS.
     * <p>
     * The columnName is the exact spelling of the name entered in an
     * addColumn call. The order this data is added will not change
     * the order of the columns.
     * <p>
     * Example: See saveInteger()
     * <p>
     * 
     * @param columnName string at the top of this column.
     * @param value      boolean to be written.
     */
    public void saveFalseBoolean(String columnName, Boolean value) { // we only want false or blank, easier to read.
        saveString(columnName, (value == true) ? "" : value.toString());
    }

    // save trace data into the log
    public void saveTrace(String traceMsg) {
        if (this.savedRows >= this.maxRows)
            return;

        double timeNow = timerStart.get();
        double deltaMs = (timeNow - timeLast) * 1000;
        listRows[this.savedRows++] = String.format("%.3f,%.1f,%s", timeNow, deltaMs, traceMsg);
    }

    /**
     * Called to tell telemetry to save the current saved
     * data into a row. This will cause the data to be saved under the
     * appropriate headers. Any columns missing data will show blank.
     * <p>
     * This is expected to be called at the bottom of the TeleopPeriodic
     * and the autonomousPeriodic methods of the Robot class.
     * <p>
     * Example:
     * <p>
     * 
     * <pre>
     *     public void autonomousPeriodic() {
     *         
     *         bla...bla...bla
     *         
     *         inputs.writeTelemetryValues(telem);
     *         robotbase.writeTelemetryValues(telem);
     *         wedge.writeTelemetryValues(telem);
     *         telem.writeRow();	                          // write a record now
     *     }
     * </pre>
     */
    public void writeRow() {
        // if( true ) return;

        // don't save beyond maximum allocation
        if (this.savedRows >= this.maxRows)
            return;

        // ordering of saves must match the default headers
        int colIndex = 0;

        // save the elapsed time
        double timeNow = timerStart.get();
        currentRowData[colIndex++] = String.format("%.3f", timeNow);

        // delta time from save writeRow, the periodic scan time
        double scanMs = (timeNow - timeLast) * 1000;
        currentRowData[colIndex++] = String.format("%.1f", scanMs);

        // update scan time histogram
        int histIndex = (int) Math.min(scanHistSize - 1, Math.round(scanMs));
        scanHist[histIndex]++;

        timeLast = timeNow;

        // time stamp the row
        currentRowData[colIndex++] = dateFormat.format(Calendar.getInstance().getTime());

        // event data
        currentRowData[colIndex++] = driverStation.getEventName();

        DriverStation.MatchType matchType = driverStation.getMatchType();
        String s_matchType = "";
        if (matchType == DriverStation.MatchType.Elimination)
            s_matchType = "Elimination";
        else if (matchType == DriverStation.MatchType.None)
            s_matchType = "None";
        else if (matchType == DriverStation.MatchType.Practice)
            s_matchType = "Practice";
        else if (matchType == DriverStation.MatchType.Qualification)
            s_matchType = "Qualification";
        currentRowData[colIndex++] = s_matchType;

        currentRowData[colIndex++] = Integer.toString(driverStation.getMatchNumber());

        currentRowData[colIndex++] = String.format("%.2f", RobotController.getBatteryVoltage());

        String s_Brownout = "";
        if (RobotController.isBrownedOut()) {
            s_Brownout = "True";
        }
        currentRowData[colIndex++] = s_Brownout;

        String s_Mode = "";

        if (driverStation.isAutonomous() == true)
            s_Mode = "auton"; // this form is allowed as Java ignores whitespace between things
        else if (driverStation.isTeleop() == true)
            s_Mode = "teleop"; // we don't like to do it as it may not be clear and harder to debug.
        else if (driverStation.isDisabled() == true)
            s_Mode = "disable"; // added here for your viewing pleasure and a learning moment

        currentRowData[colIndex++] = s_Mode; // add mode to the column data list

        // change nulls to empty
        for (colIndex = 0; colIndex < currentRowData.length; colIndex++) {
            if (null == currentRowData[colIndex])
                currentRowData[colIndex] = "";
        }

        // convert array to string
        listRows[this.savedRows++] = String.join(",", currentRowData);

        currentRowData = new String[dictColumnNames.size()];
    }

    /**
     * Called to tell telemetry to read its Preferences.
     * The available fields are...<br>
     * 1) Tele_FileName - Default: telemetry - Name of the file to be written. This
     * will have a .xls extension added.
     * <p>
     * 2) Tele_FilePath - Default: /tmp - Location on the roboRio where
     * the file is written. This is in /var/volitile/tmp.
     * This will be erased when powered off. So get it
     * before you power off. The folder /home/lvuser
     * is writable and is not volitile. <b><u><i>Use at your own risk.</i></u></b>
     * <p>
     * 3) Tele_TimestampFile - Default: false - Use to tell the system to add a
     * current time stamp to the file.
     * This is applied just before the data is written. <br>
     * Example: telemetry.2015-12-14-13-25-23.xls
     */
    public void loadConfig(Config config) {

        this.filename = config.getString("Tele_FileName", "telemetry");
        this.filePath = config.getString("Tele_FilePath", "/tmp");
        this.timeStampFilename = config.getBoolean("Tele_TimestampFile", true);
    }

    /**
     * Call see what the file name will be. You can include on
     * the SmartDashboard so you can see what the name will look like based upon
     * the preferences.<br>
     * If you have the time stamp option enabled the time will
     * be when the final file is written.
     * 
     * @return String of the full file name with .xls extension.
     */
    public String getFileName() { // build up the file name

        if (filename.length() == 0)
            filename = "telemetry";

        String s_FullFileName = filePath + "/" + filename; // + "_" + this.s_TelemType;

        if (this.timeStampFilename == true)
            s_FullFileName += "_" + new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(new Date());

        s_FullFileName += ".csv"; // .xls makes it look like an Excel spread spread

        return s_FullFileName;
    }

    /**
     * Called to show a few of the fields that can aid you in setup.<br>
     * <b>Note: This only works while in disabledPeriodic mode.</b>
     * <p>
     * 1) Tele_FileName - Final File name. Note: Is timestamp is used the
     * timestamp will updated until you actually write
     * the file.
     * <p>
     * 2) Tele_DataSaved - a boolen that will show true when the spreadsheet
     * is written. Once written you cannot write again
     * until the code is restarted.
     * <p>
     * Example: /tmp/telemetry.2015-12-14-13-25-23.xls
     * <p>
     * 
     * @param b_MinDisplay Ignored but kept for compatibility with
     *                     other LC functions with this name.
     */
    public void outputToDashBoard(Boolean b_MinDisplay) {

        if (driverStation.isDisabled()) { // only acted upon and displayed in disabled mode to save cycles
            // --SmartDashboard.putString("Tele_FileName", getFileName() );
        }
    }

    /**
     * Called to tell Telemetry to save the rows and columns
     * that are the spreadsheet data. Note: It is expected that this is called
     * while in disabledPeriodic mode during testing or at the
     * end of a competition round.
     * <p>
     * The best way is to have 2 buttons that must be pressed at the same time
     * while in disabledPeriodic mode. Possibly one on the driver stick and the
     * other on the operator stick. We do not want this to be triggered
     * accidently during a round.
     * <p>
     * Example:
     * 
     * <pre>
     * public void disabledPeriodic() {
     * 
     *     // bla..bla..bla
     * 
     *     if (inputs.b_SaveTelemetry == true)
     *         telem.saveSpreadSheet(); // once done you cannot save more data there.
     * }
     * </pre>
     * 
     */
    public void saveSpreadSheet() {

        if (this.savedRows == 0)
            return;

        try {
            FileWriter fileHandle = new FileWriter(getFileName());

            // pull the header names out of the dictionary
            String[] headerNames = new String[dictColumnNames.size()];
            for (Map.Entry<String, Integer> entry : dictColumnNames.entrySet()) {
                int index = entry.getValue();
                if (index >= 0 && index < headerNames.length) {
                    headerNames[index] = entry.getKey();
                }
            }

            // write the header. the \n is a new line indicating the end of a line or row in
            // the sheet.
            fileHandle.write(String.join(",", headerNames) + "\n");

            // write telemetry data
            for (int i = 0; i < listRows.length && listRows[i] != null; i++) {
                fileHandle.write(listRows[i] + "\n");
            }

            // write scan time histogram
            fileHandle.write("\nScan Timing Histogram\nMs,Count\n");
            int count = 0;
            int sum = 0;
            for (int i = 0; i < scanHistSize; i++) {
                if (scanHist[i] > 0) {
                    count += scanHist[i];
                    sum += i * scanHist[i];
                    fileHandle.write(String.format("%d,%d\n", i, scanHist[i]));
                }
            }
            if (count > 0) {
                fileHandle.write(String.format("Average Scan: %d\n", sum / count));
            }

            fileHandle.close();

        } catch (IOException e) {
            e.printStackTrace();
        }

        createNewList();

    }

}