package libraries.cheesylib.subsystems;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import libraries.cheesylib.util.CrashTrackingRunnable;

public class SubsystemLogManager {
    private final boolean mLoggingDisabledPermanently = false; // set this to false to permanently disable logging
    private boolean mLoggingDisabledTooManyFiles = false; // set true if logs grow too large
    private boolean mLoggingDisabledExternally = false; // set true if log not desired
    private final int mLogFileSizeLimit = 30000000; // max log file sizes allowed, arbitrary choice, robot appears to
                                                   // have ~140M free
    private final int mBufferSize = 16*1024;
    private boolean mLineEnded = true;
    private StringBuilder mLine = new StringBuilder(mBufferSize); // current line of values, estimated longest line
    private final String mBasePath = "/home/lvuser/";
    private final String mActiveLogs = "active_logs";
    private final String mCompletedLogs = "completed_logs";
    private ConcurrentLinkedDeque<String> mLinesToWrite = new ConcurrentLinkedDeque<>();

    private int mThreadPriority = Thread.NORM_PRIORITY - 1; // set below normal priority to minimize logging impact
    private Thread mThread = null;
    private Object mObject = new Object();

    private ZipOutputStream mZipOut = null;
    private FileOutputStream mFOS = null;

    public SubsystemLogManager() {
        createLogWriter();
    }

    // let's go
    public void startLogging(boolean loggingDisabled) {
        backupLogs();
        mLoggingDisabledExternally = loggingDisabled;

        // delete logs until space is available
        while (checkAvailableLogSpace()){
            if (!deleteOldestFile(mBasePath+mCompletedLogs)){
                break;
            }
        }
        
        if (loggingEnabled()) {
            System.out.println("start subsystem logging");
            mFOS = null;

            LocalDateTime myDateObj = LocalDateTime.now();
            String dateTime = myDateObj.format(DateTimeFormatter.ofPattern("yyyy-MM-dd_HH_mm_ss_SSS"));

            String nextLogFilename = "SSLog."+dateTime;

            try {
                mFOS = new FileOutputStream(mBasePath+mActiveLogs+"/"+nextLogFilename+".zip");
                System.out.println(mBasePath+mActiveLogs+"/"+nextLogFilename+".zip");
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
            mZipOut = new ZipOutputStream(mFOS);

            File fileToZip = new File(nextLogFilename+".csv");
            ZipEntry zipEntry = new ZipEntry(fileToZip.getName());
            try {
                mZipOut.putNextEntry(zipEntry);
            } catch (IOException e) {
                e.printStackTrace();
            }        
            mLineEnded = true;
            mLine.setLength(0);
        }
        else{
            mZipOut = null;
        }
    }

    // this handles open files with pending writes
    // this handles 0, 1, or more files in the active log directory
    private void backupLogs() {
        // moving previously active files to backup directory
        String[] pathnames;

        File activeDir = new File(mBasePath + mActiveLogs); // "/home/lvuser/active_logs"
        if (!activeDir.exists()) {
            activeDir.mkdir();
        }

        File completedDir = new File(mBasePath + mCompletedLogs);// "/home/lvuser/completed_logs"
        if (!completedDir.exists()) {
            completedDir.mkdir();
        }

        // close current log file
        if (mZipOut != null){
            try {
                if (mLine.length() > 0){
                    mLinesToWrite.add(mLine.toString());
                }
                writeToDisk();
                mZipOut.close();
                mZipOut = null;
                mFOS.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        // move previously active log file to completed directory
        // this will move more than one if old files exist
        pathnames = activeDir.list();

        if (pathnames != null) {
            for (String filename : pathnames) {

                File prevActiveFile = new File(activeDir.toString() + "/" + filename);

                // it appears that the file starts at 63 bytes. <100 will detect these "empty" files and delete them
                if (prevActiveFile.length() > 100) {
                    String completedFilename = prevActiveFile.toString().replace(mActiveLogs, mCompletedLogs);
                    File completedFile = new File(completedFilename);

                    if (completedFile.exists()) {
                        completedFile.delete(); // should never happen but just in case
                    }
                    prevActiveFile.renameTo(completedFile);
                } else {
                    // delete the "empty" file
                    prevActiveFile.delete();
                }
            }
        }

        moveCompletedLogsToUSB(mBasePath+mCompletedLogs);

    }

    public boolean checkAvailableLogSpace() {
        // check if log files are consuming too much space
        int logSize = fileSizes(mBasePath+mCompletedLogs); // "/home/lvuser/completed_logs"
        System.out.println("completed logs size = "+logSize);
        if (logSize > mLogFileSizeLimit){
            System.out.println("Logging disabled, log file size limit exceeded");
            mLoggingDisabledTooManyFiles = true;            
        }
        else {
            mLoggingDisabledTooManyFiles = false;
        }

        return mLoggingDisabledTooManyFiles;
    }

    // get size of all log files
    private int fileSizes(String path){
        String[] pathnames;
        int fileSizes = 0;
        File dir = new File(path);

        if (dir.exists()){
            pathnames = dir.list();

            if (pathnames != null){
                for (String filename: pathnames){
                    File file = new File(dir.toString()+"/"+filename);
                    fileSizes += file.length();
                }
            }
        }
        return fileSizes;
    }

    private boolean deleteOldestFile(String path){
        String[] pathnames;
        File dir = new File(path);
        boolean deletedAFile = false;

        if (dir.exists()){
            pathnames = dir.list();

            if (pathnames != null){
                File file = new File(dir.toString()+"/"+pathnames[0]);
                file.delete();
                deletedAFile = true;
            }
        }
        return deletedAFile;
    }

    private void moveCompletedLogsToUSB(String path){
        String[] pathnames;
        File dir = new File(path);
        File usbDir = new File("/U");
        
        if (usbDir.exists()){

            if (dir.exists()){
                pathnames = dir.list();

                if (pathnames != null){
                    for (String filename: pathnames){
                        File file = new File(dir.toString()+"/"+filename);
                        File dest = new File("/U/"+filename);
                        long fileSize = file.length();
                        try {
                            System.out.println("copy "+file.toPath().toString() +" to "+ dest.toPath().toString());
                            Files.copy(file.toPath(), dest.toPath());
                        } catch (IOException e) {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                        }
                        System.out.println("THOUGHT ABOUT moving "+file.toString() +" to "+ dest.toString());

                        if (dest.exists() && dest.length()==fileSize) {
                            file.delete();
                        }
                    }
                }
                else{
                    System.out.println("no log files to move");
                }
            }
            else{
                System.out.println("did not find log file completed directory");
            }
        }
        else{
            System.out.println("did not find usb drive");
        }
    }

    // append data from one subsystem to StringBuilder
    public void addToLine(String newLineSegment){
        if (loggingEnabled()){
            if (newLineSegment != null){
                if (!mLineEnded){
                    mLine.append(",");
                }
                else{
                    mLineEnded = false;
                }
                mLine.append(newLineSegment);
            }
        }
    }

    // called after all subsystems have reported their values in order to remove dups and add end of line char
    public void endLine(){
        if (loggingEnabled()){
            if (mZipOut != null){
                mLine.append(System.getProperty("line.separator"));
                mLineEnded = true;
                if (mLine.length() > mBufferSize*.9){
                    mLinesToWrite.add(mLine.toString());
                    synchronized(mObject){
                        mObject.notifyAll();
                    }
                    mLine.setLength(0);
                }
            }
        }
    }

    private synchronized void writeToDisk(){
        try {
            if (mZipOut != null) {
                while (true) {
                    String val = mLinesToWrite.pollFirst();
                    if (val == null) {
                        break;
                    }
                    mZipOut.write(val.getBytes());
                    mZipOut.flush();
                }
            }    
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    private boolean loggingEnabled(){
        return !mLoggingDisabledPermanently && !mLoggingDisabledTooManyFiles && !mLoggingDisabledExternally;
    }

    // create thread to regularly flush buffered writes to disk
    private void createLogWriter() {
        if (mThreadPriority == 0) {
            mThreadPriority = Thread.NORM_PRIORITY;
        } else {
            mThreadPriority = Math.max(Math.min(Thread.MAX_PRIORITY, mThreadPriority), Thread.MIN_PRIORITY);
        }

        mThread = new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                while (true) {
                    try {
                        synchronized(mObject){
                            mObject.wait();
                        }
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                        Thread.currentThread().interrupt();
                    }
                    writeToDisk();
                }
            }
        });
        mThread.setPriority(mThreadPriority);
        mThread.start();
    }
}