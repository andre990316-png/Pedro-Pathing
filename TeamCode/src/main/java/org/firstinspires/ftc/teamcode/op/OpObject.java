package org.firstinspires.ftc.teamcode.op;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.HashSet;

public abstract class OpObject extends OpMode {
    private final static HashSet<OpStruct> globalstructs = new HashSet<>();
    public static void addStruct(OpStruct struct){
        globalstructs.add(struct);
    }

    public abstract void Init();
    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();

    @Override
    public final void init() {
        Init();
        for (OpStruct o : globalstructs) {
            o.Init();
        }
    }
    @Override
    public final void start() {
        Start();
        for (OpStruct o : globalstructs) {
            o.Start();
        }
    }
    @Override
    public final void loop() {
        Loop();
        for (OpStruct o : globalstructs) {
            o.Loop();
        }
    }
    @Override
    public final void stop() {
        Stop();
        for (OpStruct o : globalstructs) {
            o.Stop();
        }
    }





}
