package frc.robot;

import frc.framework.FieldReference;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.framework.DataPoller;
import frc.framework.Executor;
import frc.framework.ProductionManager;

public class ShooterSubsystem implements DataPoller, Executor {
    public static final FieldReference<AngularVelocity> TARGET_FLYWHEEL_SPEED = new FieldReference<AngularVelocity>(RadiansPerSecond.of(0));

    public AngularVelocity flywheelSpeed;
    public TalonFX motor;

    public VelocityVoltage motorRequest = new VelocityVoltage(0);

    public ShooterSubsystem(TalonFX motor) {
        this.motor = motor;
    }

    @Override
    public void updateData() {
        flywheelSpeed = motor.getVelocity().getValue();
    }

    @Override
    public void execute(ProductionManager manager) {
        //set flywheelspeed
        motorRequest.withVelocity(manager.getValue(TARGET_FLYWHEEL_SPEED));

        motor.setControl(motorRequest);
    }
}
