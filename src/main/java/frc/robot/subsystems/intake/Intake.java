// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.lang.System.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Intake extends SubsystemBase {
    public static IntakeIO intakeIO;
    // public static IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        System.out.println("[Init] Creating Intake ");
        intakeIO = io;
    }

    public void periodic() {
        // intakeIO.updateInputs(intakeInputs);
        // Logger.getInstance().processInputs("Intake", intakeInputs);
    }

}
