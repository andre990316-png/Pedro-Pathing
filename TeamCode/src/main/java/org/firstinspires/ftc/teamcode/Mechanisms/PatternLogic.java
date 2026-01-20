package org.firstinspires.ftc.teamcode.Mechanisms;
import java.util.Vector;
public class PatternLogic {
    private Vector<String> box = new Vector<>();
    private Vector<String> pattern = new Vector<>();
    private Integer green, purple = 0;
    public void initPattern(int id) {
        switch (id) {
            case 21:
                pattern.add("G");
                pattern.add("P");
                pattern.add("P");
            case 22:
                pattern.add("P");
                pattern.add("G");
                pattern.add("P");
            case 23:
                pattern.add("P");
                pattern.add("P");
                pattern.add("G");
        }
    }
    public void addArtifact(String color) {
        box.add(color);
        switch (color) {
            case "G":
                green++;
            case "P":
                purple++;
        }
    }
}
