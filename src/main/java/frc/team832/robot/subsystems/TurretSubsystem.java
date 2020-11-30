package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.PDPSlot;
import frc.team832.lib.sensors.REVThroughBorePWM;
import frc.team832.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    public final boolean initSuccessful;

    private final CANSparkMax motor;
    private final REVThroughBorePWM encoder;
    private final PDPSlot pdpSlot;

    private NetworkTableEntry dashboard_turretPos, dashboard_turretPow, dashboard_turretTarget;

    public TurretSubsystem(GrouchPDP pdp) {
        setName("Turret");
        DashboardManager.addTab(this);
        motor = new CANSparkMax(Constants.TurretValues.TURRET_MOTOR_CAN_ID, Motor.kNEO550);
        encoder = new REVThroughBorePWM(Constants.TurretValues.TURRET_ENCODER_DIO_CHANNEL);
        pdpSlot = pdp.addDevice(Constants.TurretValues.TURRET_PDP_SLOT, motor);

        motor.wipeSettings();

        motor.limitInputCurrent(25);
        motor.setNeutralMode(NeutralMode.kBrake);

        dashboard_turretPos = DashboardManager.addTabItem(this, "Position", 0.0);
        dashboard_turretPow = DashboardManager.addTabItem(this, "Power", 0.0);
        dashboard_turretTarget = DashboardManager.addTabItem(this, "Target", 0.0);

        initSuccessful = motor.getCANConnection();
    }

    public void updateControlLoops() {
        // run PID here
    }

    @Override
    public void periodic() {
        updateDashboardData();
    }

    public void updateDashboardData() {
//        dashboard_turretPos.setDouble(getDegrees());
        dashboard_turretPow.setDouble(motor.getOutputVoltage());
//        dashboard_turretTarget.setDouble(turretTargetDeg);
    }
}
