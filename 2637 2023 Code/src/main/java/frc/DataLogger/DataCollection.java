package frc.DataLogger;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.String;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

public class DataCollection 
{	
    Date date;	
    SimpleDateFormat sdf;
    String dateFormatted;

    public boolean fileNotAppended = false;

    public final String logDataFilePath = "//media//sda1//RobotData";
    public final String logDataFileType = ".csv";

    public        boolean logDataValues = false;   
    public static int     logDataID; 

    public ArrayList<CatzLog> logData; 

    StringBuilder sb = new StringBuilder();

    public static final int LOG_ID_NONE    = 0;
    public static final int LOG_ID_BALANCE = 1;
    public static final int LOG_ID_INDEXER = 2;
    public static final int LOG_ID_SWERVE_DRIVING = 3;
    public static final int LOG_ID_SWERVE_STEERING = 4;
    public boolean validLogID = true;

    private final String LOG_HDR_BALANCE_MOD = "1,2,3,4";
    public String logStr;

    public static String mechanismName = "Not Set";

    public static final SendableChooser<Integer> chosenDataID = new SendableChooser<>();

    public static int boolData = 0;

    public static final int shift0 = 1 << 0;
    public static final int shift1 = 1 << 1;
    public static final int shift2 = 1 << 2;
    public static final int shift3 = 1 << 3;
    public static final int shift4 = 1 << 4;
    public static final int shift5 = 1 << 5;
    public static final int shift6 = 1 << 6;
    public static final int shift7 = 1 << 7;

    public void setLogDataID(final int dataID)
    {
        logDataID = dataID;
    }

    
    public void dataCollectionInit(final ArrayList<CatzLog> list)
    {   
        date = Calendar.getInstance().getTime();
      
        sdf = new SimpleDateFormat("yyyy-MM-dd kk.mm.ss");	
        dateFormatted = sdf.format(date);
        logData = list;

        dataCollectionShuffleboard();
    }

    /*-----------------------------------------------------------------------------------------
    *  Initialize drop down menu for data collection on Shuffleboard
    *----------------------------------------------------------------------------------------*/
    public void dataCollectionShuffleboard()
    {
        chosenDataID.setDefaultOption("None",        LOG_ID_NONE);
        chosenDataID.addOption("Balance Data",       LOG_ID_BALANCE);
        chosenDataID.addOption("INDEXER", LOG_ID_INDEXER);

        SmartDashboard.putData("Data Collection", chosenDataID);
    }

    public void startDataCollection() 
    {
        logDataValues = true;
    }

    public void stopDataCollection() 
    {
        logDataValues = false; 
    }

    /* ---------------------------------------------------
    Method only used for mechanisms with no running threads
    ------------------------------------------------------*/
    public static void resetBooleanData()
    {
        boolData = 0;
    }

    public static void booleanDataLogging(boolean bool1, int bitPos)
    {
        if(bool1 == true)
        {
            boolData |= (1 << bitPos);
        }
    }
    
    public void writeHeader(PrintWriter pw) 
    {
        switch (logDataID)
        {
            case LOG_ID_BALANCE :

             pw.printf(LOG_HDR_BALANCE_MOD);
             break;

            default :
                pw.printf("Invalid Log Data ID");            


        }
    }
    
    public String createFilePath()
    {
        String logDataFullFilePath = logDataFilePath + " " + setLogDataName() + " " + dateFormatted +  logDataFileType;
    	return logDataFullFilePath;
    }

    public void exportData(ArrayList<CatzLog> data) throws IOException
    {   
        System.out.println("Export Data ///////////////");    
        try (
            
        FileWriter     fw = new FileWriter(createFilePath(), fileNotAppended);
        BufferedWriter bw = new BufferedWriter(fw);
        PrintWriter    pw = new PrintWriter(bw))

        {
            writeHeader(pw);
            pw.print("\n");

    
            int dataSize = data.size();
            for (int i = 0; i < dataSize; i++)
            {
                pw.print(data.get(i).toString() + "\n");
                pw.flush();
            }

            pw.close();
        }
    }

    public static int getLogDataID()
    {
        return logDataID;
    }

    public static String setLogDataName()
    {
        switch(getLogDataID())
        {

            case(LOG_ID_INDEXER):
            mechanismName = "Index";
                break;
        }
        return mechanismName;
    }
}