package frc.robot.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.indexerConstants;

// 1 indexer 1amp roller
public class indexer extends SubsystemBase {
    private final TalonFX indexer = new TalonFX(canIDConstants.indexerMotor, "canivore");
    private final TalonFX ampRoller = new TalonFX(canIDConstants.ampRollerMotor, "rio");

    private TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
    private TalonFXConfiguration ampRollerConfigs = new TalonFXConfiguration();

    private VoltageOut indexerVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    private VoltageOut ampRollerVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    
    private final StatusSignal<Double> indexerCurrent= indexer.getStatorCurrent();
    private final StatusSignal<Double> indexerTemp = indexer.getDeviceTemp();
    private final StatusSignal<Double> indexerRPS = indexer.getRotorVelocity();

    private final StatusSignal<Double> ampRollerCurrent= ampRoller.getStatorCurrent();
    private final StatusSignal<Double> ampRollerTemp = indexer.getDeviceTemp();
    private final StatusSignal<Double> ampRollerRPS = indexer.getRotorVelocity();
    
    private double indexerSetpointVolts;
    private double ampRollerSetpointVolts;

    public indexer(){
        indexerConfigs.CurrentLimits.StatorCurrentLimit = indexerConstants.indexerCurrentLimit;
        indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerConfigs.MotorOutput.Inverted = indexerConstants.indexerInvert;

        ampRollerConfigs.CurrentLimits.StatorCurrentLimit = indexerConstants.ampRollerCurrentLimit;
        ampRollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        ampRollerConfigs.MotorOutput.Inverted = indexerConstants.ampRollerInvert;

        indexer.getConfigurator().apply(indexerConfigs);
        ampRoller.getConfigurator().apply(ampRollerConfigs);
        
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            indexerCurrent,
            indexerTemp,
            indexerRPS,
            ampRollerCurrent,
            ampRollerTemp,
            ampRollerRPS
            );

        indexer.optimizeBusUtilization();
        ampRoller.optimizeBusUtilization();

        indexerSetpointVolts = 0.0;
        ampRollerSetpointVolts = 0.0;
    }
public void periodic(){
    BaseStatusSignal.refreshAll(
        indexerCurrent,
        indexerTemp,
        indexerRPS,
        ampRollerCurrent,
        ampRollerTemp,
        ampRollerRPS
    );

SmartDashboard.putNumber("Indexer Current", indexerCurrent.getValue());
SmartDashboard.putNumber("Indexer RPS", indexerRPS.getValue());
SmartDashboard.putNumber("Indexer Temp", indexerTemp.getValue());
SmartDashboard.putNumber("Amp Current", ampRollerCurrent.getValue());
SmartDashboard.putNumber("Amp RPS", ampRollerRPS.getValue());
SmartDashboard.putNumber("Amp Temp", ampRollerTemp.getValue());
    }

    public void requestIndexerVoltage(double voltage){
        indexer.setControl(indexerVoltageRequest.withOutput(voltage));
    }

    public void requestAmpRollerVoltage(double voltage){
        ampRoller.setControl(ampRollerVoltageRequest.withOutput(voltage));
    }



}
