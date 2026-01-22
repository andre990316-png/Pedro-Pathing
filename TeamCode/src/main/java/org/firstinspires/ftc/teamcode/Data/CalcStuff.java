package org.firstinspires.ftc.teamcode.Data;

public class CalcStuff {
    public static double iwrotethiswithoutchatgptyouguysarenooooobs(double[] ranges, double[] values, double val){
        /// preconditions:
        /// ranges[] and values[] must have the same length,
        /// and be in ascending order
        int index=0;
        double percentage=0;
        double low=0;
        double high=0;
        if(val<=ranges[0]){
            return values[0];
        }
        if(val>=ranges[ranges.length-1]){
            return values[ranges.length-1];
        }
        for(int i=0; i<ranges.length-2; i++){
            if(values[i]<=val&&values[i+1]>=val){
                index=i;
                low=values[i];
                high=values[i+1];
                percentage=(val-low)/(high-low);
                break;
            }
        }
        return low+(high-low)*percentage;

    }
}
