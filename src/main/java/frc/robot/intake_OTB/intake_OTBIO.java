package frc.robot.intake_OTB;

import org.littletonrobotics.junction.AutoLog;

public interface intake_OTBIO {
    @AutoLog
    public static class intake_OTBIOInputs {
        public double intakeCurrent = 0.0;
        public double intakeTemp = 0.0;
        public double intakeRPS = 0.0;
        public double setpointVolts = 0.0;
        public double pivotCurrent = 0.0;
        public double pivotTemp = 0.0;
        public double pivotRPS = 0.0;
        public double pivotSetpointDeg = 0.0;
    }
    public void updateInputs(intake_OTBIOInputs inputs);

    public void setPivotVoltage(double voltage);

    public void setPivotPosition(double angleDegrees);

    public void setIntakeVoltage(double voltage);

    public void zeroPosition();


}
