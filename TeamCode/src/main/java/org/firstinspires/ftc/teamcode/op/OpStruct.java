package org.firstinspires.ftc.teamcode.op;

public abstract class OpStruct {
    protected OpObject m_object;
    public abstract void Init();
    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();
}
