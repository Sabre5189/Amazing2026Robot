// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeArmSubsystem extends SubsystemBase {

    private final SparkMax motor;
    private final DutyCycleEncoder absoluteEncoder;
    private final PIDController pid;
 
    public IntakeArmSubsystem() {

        // Motor
        motor = new SparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);

        SparkMaxConfig configs = new SparkMaxConfig();       
        motor.configure(configs,
               ResetMode.kResetSafeParameters,
               PersistMode.kPersistParameters);

        // Absolute encoder on DIO
        absoluteEncoder = new DutyCycleEncoder(IntakeConstants.kEncoderDIO);

        // WPILib PID (since encoder is on RIO)
        pid = new PIDController(
                IntakeConstants.kP,
                IntakeConstants.kI,
                IntakeConstants.kD
        );

        pid.setTolerance(0.01);
        // Enable continuous input (important for absolute encoders)
       pid.enableContinuousInput(0.0, 1.0);
    }

    // Get absolute position (0.0 - 1.0 rotations)
    public double getAbsolutePosition() {
        double position =-absoluteEncoder.get();

        position -= IntakeConstants.kEncoderOffset;

        // Wrap between 0 and 1
        return MathUtil.inputModulus(position, 0.0, 1.0);
    }

    // Convert to degrees (easier for students)
    public double getDegrees() {
        return getAbsolutePosition() * 360.0;
    }

    // Open loop
    public void run(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    // Closed-loop position control (degrees)
    public void moveToAngle(double targetDegrees) {

        double targetRotations = targetDegrees / 360.0;

        double output = pid.calculate(getAbsolutePosition(), targetRotations);

        output = MathUtil.clamp(output, -0.5, 0.5);  // start small
        
        motor.set(output);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // This method will be called once per scheduler run during simulation
 SmartDashboard.putNumber("Current Position", getDegrees());
    Logger.recordOutput("INTAKEARM/Current Position", getDegrees());
    Logger.recordOutput("INTAKEARM/Target Position", 90);
   // Logger.recordOutput()
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
 SmartDashboard.putNumber("Current Position", absoluteEncoder.get());
    Logger.recordOutput("INTAKEARM/Current Position", absoluteEncoder.get());
   // Logger.recordOutput()
  }
}
