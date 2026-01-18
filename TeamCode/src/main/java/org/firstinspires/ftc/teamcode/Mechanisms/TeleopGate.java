package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleopGate {

    private Servo gateServo;
    private ElapsedTime timer = new ElapsedTime();
    private boolean gateOpen = false;

    private FlywheelLogic flywheel;

    // Servo positions
    private static final double OPEN = 0.0;
    private static final double CLOSED = 0.2;

    // Frequencies in Hz
    private static final double LOW_RPM_FREQ = 224.0 / 60.0;  // 224 times per minute
    private static final double HIGH_RPM_FREQ = 80.0 / 60.0;  // 80 times per minute

    private double period = 0.0;
    private double rpmRange = 400.0;

    public TeleopGate(Servo servo) {
        gateServo = servo;
        gateServo.setPosition(CLOSED);
    }

    public void update(double targetRPM, boolean triggerPressed) {
        if (!triggerPressed) {
            // Close gate if trigger is released
            gateServo.setPosition(CLOSED);
            gateOpen = false;
            timer.reset();
            return;
        }

        // Choose period based on RPM
        period = (targetRPM <= 4500) ? 1.0 / LOW_RPM_FREQ : 1.0 / HIGH_RPM_FREQ;

        // Toggle gate if period elapsed
        if (flywheel.getError() <= rpmRange){
            if (timer.seconds() >= period / 2.0) {  // divide by 2 because each cycle = open+close
                gateOpen = !gateOpen;
                gateServo.setPosition(gateOpen ? OPEN : CLOSED);
                timer.reset();
            }
        } else {
            gateServo.setPosition(CLOSED);
        }

    }
}
