package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.PDPSlot;
import frc.team832.lib.sensors.REVThroughBorePWM;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants.TurretValues;

public class TurretSubsystem extends SubsystemBase {

    public final boolean initSuccessful;

    private double turretTargetDeg = 0;

    private final VisionSubsystem vision;

    private final CANSparkMax turretMotor;
    private final REVThroughBorePWM encoder;
    private final PDPSlot pdpSlot;
    private final ProfiledPIDController pid;

    private NetworkTableEntry dashboard_turretPos, dashboard_turretPIDEffort, dashboard_turretTarget;

    public TurretSubsystem(GrouchPDP pdp, VisionSubsystem vision) {
        setName("Turret");
        this.vision = vision;

        DashboardManager.addTab(this);
        turretMotor = new CANSparkMax(TurretValues.TURRET_MOTOR_CAN_ID, Motor.kNEO550);
        encoder = new REVThroughBorePWM(TurretValues.TURRET_ENCODER_DIO_CHANNEL);
        pdpSlot = pdp.addDevice(TurretValues.TURRET_PDP_SLOT, turretMotor);

        turretMotor.wipeSettings();

        turretMotor.limitInputCurrent(25);
        turretMotor.setNeutralMode(NeutralMode.kBrake);

        pid = new ProfiledPIDController(TurretValues.kP, TurretValues.kI, TurretValues.kD, TurretValues.Constraints);
        pid.setIntegratorRange(-0.1, 0.1);

        dashboard_turretPos = DashboardManager.addTabItem(this, "Position", 0.0);
        dashboard_turretPIDEffort = DashboardManager.addTabItem(this, "Effort", 0.0);
        dashboard_turretTarget = DashboardManager.addTabItem(this, "Target", 0.0);

        initSuccessful = turretMotor.getCANConnection();
    }

    public void updateControlLoops() {
          runTurretPid();
    }

    private void runTurretPid() {
        turretMotor.set(pid.calculate(getDegrees(), turretTargetDeg));
    }

    public void updateDashboardData() {
        dashboard_turretPos.setDouble(getDegrees());
        dashboard_turretPIDEffort.setDouble(pid.calculate(getDegrees(), turretTargetDeg));
        dashboard_turretTarget.setDouble(turretTargetDeg);
    }

    public void setTurretTargetDegrees(double pos) {
        turretTargetDeg = pos;
    }

    public void trackTarget() {
        double visionOffset = 0;
        if (vision.hasTarget()) visionOffset = vision.getTarget().getYaw();

        setTurretTargetDegrees(visionOffset + getDegrees());
    }

    double getRotations() {
        return OscarMath.round(encoder.get(), 3);
    }

    double getDegrees() {
        return TurretValues.convertRotationsToDegrees(getRotations()) + 160;
    }

    public void setForward() { setTurretTargetDegrees(TurretValues.TurretCenterVisionPosition); }

    public void setIntake() {
        setTurretTargetDegrees(0);
    }

    @Override
    public void periodic() {
        updateDashboardData();
    }

    public void setNeutralMode(NeutralMode mode) {
        turretMotor.setNeutralMode(mode);
    }
}
