package libraries.cheesylib.util;

import java.lang.reflect.Field;

/**
 * Writes data to a CSV file
 */
public class ReflectingCSVFormatter<T> {
    Class<T> mTypeClass;
    private Field[] mFields;
    StringBuilder mHeaders = null; // an empty PeriodicIO will return null

    public ReflectingCSVFormatter(Class<T> typeClass) {
        this(typeClass, "");
    }

    public ReflectingCSVFormatter(Class<T> typeClass, String identifier) {
        mTypeClass = typeClass;
        mFields = typeClass.getFields();
        buildHeaders(identifier);
    }

    private void buildHeaders(String identifier) {
        if (mFields.length != 0){
            String[] classNames = mTypeClass.getEnclosingClass().toString().split("\\.");
            String shortName = classNames[classNames.length-1]+identifier;
            mHeaders = new StringBuilder();
        
            for (Field field : mFields) {
                if (mHeaders.length() != 0) {
                    mHeaders.append(",");
                }
                mHeaders.append(shortName+"."+field.getName());
            }
        }
    }

    public StringBuilder getHeaders(){
        return mHeaders;
    }

    // will return null if PeriodicIO is empty
    public StringBuilder getValues(T value) {
        // StringBuilder line = null;

        // if (mFields.length != 0){
        //     line = new StringBuilder();

        //     for (Field field : mFields) {
        //         if (line.length() != 0) {
        //             line.append(",");
        //         }
        //         try {
        //             String tmp;
        //             if (CSVWritable.class.isAssignableFrom(field.getType())) {
        //                 tmp = ((CSVWritable) field.get(value)).toCSV();
        //             } else {
        //                 tmp = field.get(value).toString();
        //             }
        //             if (!tmp.contains(",")) {
        //                 line.append(tmp);
        //             }
        //             else{
        //                 line.append("\""+tmp+"\"");
        //             }
        //         } catch (IllegalArgumentException | IllegalAccessException e) {
        //             e.printStackTrace();
        //         }
        //     }
        // }
        return mHeaders;
        // return line;
    }
}
