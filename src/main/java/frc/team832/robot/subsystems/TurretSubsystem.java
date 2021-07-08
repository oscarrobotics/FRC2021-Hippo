package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.util.Units;
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

    private double turretTargetDeg;
    private boolean isVision;
    private double turretFF = 0;

    private final VisionSubsystem vision;

    private final CANSparkMax turretMotor;
    private final REVThroughBorePWM encoder;
    private final PDPSlot pdpSlot;
    private final ProfiledPIDController pidController;

    private NetworkTableEntry dashboard_turretPos, dashboard_turretPow, dashboard_turretTarget;

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

        pidController = new ProfiledPIDController(TurretValues.kP, 0, TurretValues.kD, TurretValues.Constraints);

        dashboard_turretPos = DashboardManager.addTabItem(this, "Position", 0.0);
        dashboard_turretPow = DashboardManager.addTabItem(this, "Power", 0.0);
        dashboard_turretTarget = DashboardManager.addTabItem(this, "Target", 0.0);

        initSuccessful = turretMotor.getCANConnection();
    }

    public void updateControlLoops() {
        if (RobotState.isEnabled()) {
            var currentDegrees = TurretValues.convertRotationsToDegrees(encoder.get());
            var currentRads = Units.degreesToRadians(currentDegrees);
            var errorDegrees = turretTargetDeg - currentDegrees;
//            var cur = encoder.get();
//            var errorDeg = turretTargetDeg - cur;
//            var effort = (TurretValues.kP * errorDeg) + TurretValues.turretFF.calculate();
//            setVoltage(effort);
        }
    }

    public void setTurretTargetDegrees(double pos, boolean visionMode) {
        isVision = visionMode;
        turretTargetDeg = pos;
    }

    public void trackTarget() {
        double visionOffset = 0;
        if (vision.hasTarget()) visionOffset = vision.getTarget().getYaw();

        setTurretTargetDegrees(visionOffset + getDegrees(), true);
    }

    double getRotations() {
        return OscarMath.round(encoder.get(), 3);
    }

    double getDegrees() {
        return TurretValues.convertRotationsToDegrees(getRotations());
    }

    public void setForward(boolean isVision) { setTurretTargetDegrees(TurretValues.TurretCenterVisionPosition, isVision); }

    public void setIntake() {
        setTurretTargetDegrees(0, false);
    }

    @Override
    public void periodic() {
        updateDashboardData();
    }

    private void setVoltage(double voltage){
        turretMotor.set(voltage / turretMotor.getInputVoltage());
    }

    public void updateDashboardData() {
//        dashboard_turretPos.setDouble(getDegrees());
        dashboard_turretPow.setDouble(turretMotor.getOutputVoltage());
//        dashboard_turretTarget.setDouble(turretTargetDeg);
    }

    public void setNeutralMode(NeutralMode mode) {
        turretMotor.setNeutralMode(mode);
    }
}
