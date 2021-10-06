package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

public class ConsistentJavaStuff {

    //Consistent Variables
    private static double TEST = 1;

    //Setter Method
    public static void setTEST(double input){
        TEST = input;
    }

    //Getter Method
    public static double getTest(){
        return TEST;
    }

    //Test writing Data to file
    public static void writeData(){
        String filename = "Gaming.txt";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, String.valueOf(TEST));
    }
}
