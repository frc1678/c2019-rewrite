/*test interface*/
package com.team254.lib.util;
import java.util.ArrayList;
public class TestLoggable implements ILoggable {
    public ArrayList<ArrayList<Double>> get_items() {
        ArrayList<ArrayList<Double>> arrlist = new ArrayList<ArrayList<Double>>();
        ArrayList<Double> arr = new ArrayList<Double>();
        arr.add(1.0);
        arr.add(2.0);
        arr.add(3.0);
        arrlist.add(arr);
        return(arrlist);
    }
    public ArrayList<String> get_item_names() {
        ArrayList<String> ar = new ArrayList<String>();
        ar.add("var3");
        ar.add("var2");
        ar.add("var1");
        return(ar);
    }
}
