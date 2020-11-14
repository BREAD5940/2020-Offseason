package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class controls Obi Non's intake. The intake has a long piston to move the intake up/down,
 * a shorter piston to move the second stage, and a NEO w/CAN Spark Max spins the rollers. 
 * 
 * Holds hardware necessary to control the intake
 */
public class IntakeSubsystem extends SubsystemBase {
    DoubleSolenoid backPiston = new DoubleSolenoid(1, 2);

    public void extendBack() {
        backPiston.set(Value.kForward);
    }

    public void retractBack() {
        backPiston.set(Value.kReverse);
    }
}
