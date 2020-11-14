/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This shooter spins to shoot balls
 * We have attached a NEO motor with a CAN spark MAX
 * We can measure speed with a through-bore encoder
 */
public class Robot extends TimedRobot {

  Encoder encoder = new Encoder(0, 1);
  CANSparkMax motor = new CANSparkMax(10, MotorType.kBrushless);

  @Override
  public void teleopPeriodic() {
    motor.set(1);
    encoder.setDistancePerPulse(4096);
    double flywheelSpeed = encoder.getRate();
    System.out.println(flywheelSpeed);
    
    SmartDashboard.putNumber("Flywheel Speed", flywheelSpeed);
  }

}
