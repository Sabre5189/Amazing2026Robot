// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;



public class IntakeArmSubsystem extends SubsystemBase {
// https://github.com/REVrobotics/REVLib-Examples/blob/main/Java/SPARK/MAXMotion/src/main/java/frc/robot/Robot.java
     
   private final SparkMax intakeArm = new SparkMax(9, MotorType.kBrushless);
    private SparkClosedLoopController pidController;
    private final DutyCycleEncoder dencoder = new DutyCycleEncoder(4); 
    private SparkMaxConfig armMotorConfig;
    private double degree;
    public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  /** Creates a new ExampleSubsystem. */
  public IntakeArmSubsystem() {

    pidController = intakeArm.getClosedLoopController();
    armMotorConfig = new SparkMaxConfig();
      
    armMotorConfig.absoluteEncoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

            /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    armMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0
        .p(0.5)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-.5, .5, ClosedLoopSlot.kSlot1).feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

          armMotorConfig.closedLoop.maxMotion
          .cruiseVelocity(10)
          .maxAcceleration(1000)
          .allowedProfileError(.01);

    intakeArm.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command setReference(double rpm) {
    return run(() -> {
      pidController.setSetpoint(rpm, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // This method will be called once per scheduler run during simulation
 SmartDashboard.putNumber("Current Position", dencoder.get());
    Logger.recordOutput("INTAKEARM/Current Position", dencoder.get());
    Logger.recordOutput("INTAKEARM/Target Position", degree);
   // Logger.recordOutput()
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
 SmartDashboard.putNumber("Current Position", dencoder.get());
    Logger.recordOutput("INTAKEARM/Current Position", dencoder.get());
    Logger.recordOutput("INTAKEARM/Target Position", degree);
   // Logger.recordOutput()
  }
}
