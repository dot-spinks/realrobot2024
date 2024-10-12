// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.indexer.indexer;
import frc.robot.shooter.shooter;
import frc.robot.intake_OTB.intake_OTB;

public class RobotContainer {
    public final CommandXboxController controller = new CommandXboxController(0);
    private intake_OTB intake = new intake_OTB();
    private shooter shooter = new shooter();
     private indexer indexer = new indexer();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
   controller.leftTrigger().whileTrue((new InstantCommand(() -> intake.requestIntake(20, 2))));

    controller.rightTrigger().whileTrue(new InstantCommand(() -> shooter.requestVelocity(10, 1)));
    controller.a().whileTrue(new InstantCommand(() -> shooter.requestSetpoint(90)));

    controller.x().whileTrue(new InstantCommand(() -> indexer.requestIndexerVoltage(2)));
    controller.y().whileTrue(new InstantCommand(() -> indexer.requestAmpRollerVoltage(2)));
  
  }
    public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");

  }
}

/*    

 public void requestIndexerVoltage(double voltage){
        indexer.setControl(indexerVoltageRequest.withOutput(voltage));
    }

    public void requestAmpRollerVoltage(double voltage){
        ampRoller.setControl(ampRollerVoltageRequest.withOutput(voltage));
    }
    */