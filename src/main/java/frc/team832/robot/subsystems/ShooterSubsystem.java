package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.sensors.REVThroughBoreRelative;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants.ShooterValues;

@SuppressWarnings("FieldCanBeLocal")
public class ShooterSubsystem extends SubsystemBase {
    public final boolean initSuccessful;

    private final VisionSubsystem vision;

    private ProfiledPIDController hoodPID = new ProfiledPIDController(ShooterValues.HoodkP, 0, 0, ShooterValues.HoodConstraints);

    private double feedTargetRPM, hoodTargetDegrees, hoodDegrees, flywheelTargetRPM;

    private final CANSparkMax primaryMotor, secondaryMotor, feederMotor, hoodMotor;

    private final NetworkTableEntry dashboard_wheelRPM, dashboard_flywheelFF, dashboard_hoodCount, dashboard_hoodAngle, dashboard_wheelTargetRPM,
            dashboard_feedWheelRPM, dashboard_feedWheelTargetRPM, dashboard_feedFF;

    private final REVThroughBoreRelative flywheelEncoder = new REVThroughBoreRelative(0, 1);

    @SuppressWarnings("unused")
    private final SmartMCAttachedPDPSlot primaryFlywheelSlot, secondaryFlywheelSlot, feederSlot, hoodMotorSlot;

    public ShooterSubsystem(GrouchPDP pdp, VisionSubsystem vision) {
        DashboardManager.addTab(this);

        this.vision = vision;

        primaryMotor = new CANSparkMax(ShooterValues.PRIMARY_CAN_ID, Motor.kNEO);
        secondaryMotor = new CANSparkMax(ShooterValues.SECONDARY_CAN_ID, Motor.kNEO);
        feederMotor = new CANSparkMax(ShooterValues.FEED_MOTOR_CAN_ID, Motor.kNEO);
        hoodMotor = new CANSparkMax(ShooterValues.HOOD_MOTOR_CAN_ID, Motor.kNEO550);

        primaryFlywheelSlot = pdp.addDevice(ShooterValues.PRIMARY_PDP_SLOT, primaryMotor);
        secondaryFlywheelSlot = pdp.addDevice(ShooterValues.SECONDARY_PDP_SLOT, secondaryMotor);
        feederSlot = pdp.addDevice(ShooterValues.FEEDER_PDP_SLOT, feederMotor);
        hoodMotorSlot = pdp.addDevice(ShooterValues.HOOD_MOTOR_PDP_SLOT, hoodMotor);

        primaryMotor.wipeSettings();
        secondaryMotor.wipeSettings();
        feederMotor.wipeSettings();
        hoodMotor.wipeSettings();

        setFlyheelNeutralMode(NeutralMode.kCoast);
        setFeederNeutralMode(NeutralMode.kBrake);
        hoodMotor.setNeutralMode(NeutralMode.kBrake);

        primaryMotor.setInverted(false);
        secondaryMotor.follow(primaryMotor, true);

        primaryMotor.limitInputCurrent(55);
        secondaryMotor.limitInputCurrent(55);
        feederMotor.limitInputCurrent(25);
        hoodMotor.limitInputCurrent(15); //CHANGE NUMBER

        zeroHood();

        // dashboard
        dashboard_wheelRPM = DashboardManager.addTabItem(this, "Flywheel/RPM", 0.0);
        dashboard_wheelTargetRPM = DashboardManager.addTabItem(this, "Flywheel/Target RPM", 0.0);
        dashboard_flywheelFF = DashboardManager.addTabItem(this, "Flywheel/FF", 0.0);
        dashboard_feedWheelRPM = DashboardManager.addTabItem(this, "Feeder/RPM", 0.0);
        dashboard_feedWheelTargetRPM = DashboardManager.addTabItem(this, "Feeder/Target RPM", 0.0);
        dashboard_feedFF = DashboardManager.addTabItem(this, "Feeder/FF", 0.0);
        dashboard_hoodCount = DashboardManager.addTabItem(this, "Hood/Position", 0.0);
        dashboard_hoodAngle = DashboardManager.addTabItem(this, "Hood/Angle", 0.0);

        initSuccessful = primaryMotor.getCANConnection() && secondaryMotor.getCANConnection() && feederMotor.getCANConnection();
    }

    public void updateDashboardData() {
        dashboard_wheelRPM.setDouble(primaryMotor.getSensorVelocity() * ShooterValues.FlywheelReduction);
        dashboard_feedWheelRPM.setDouble(feederMotor.getSensorVelocity());
        dashboard_hoodAngle.setDouble(hoodDegrees);
        dashboard_hoodCount.setDouble(hoodMotor.getSensorPosition());
    }

    @Override
    public void periodic(){
        updateDashboardData();
    }

    public void zeroHood() {
        hoodMotor.rezeroSensor();
    }

    private void runFlywheelPid() {
    }

    private void runHoodPid() {
        hoodDegrees = getHoodDegrees();
        var hoodEffort = hoodPID.calculate(hoodDegrees, hoodTargetDegrees);
        hoodMotor.set(hoodEffort / RobotController.getBatteryVoltage());
    }

    public void setFeedRPM(double rpm) {
        feedTargetRPM = rpm;
    }

    public void setFlywheelRPM(double wheelTargetRPM) {
        flywheelTargetRPM = wheelTargetRPM;
    }

    public void setHoodAngle(double degrees) {
        hoodTargetDegrees = degrees;
    }

    private double getHoodDegrees() {
        return hoodCountToDegrees(hoodMotor.getSensorPosition());
    }

    private double hoodCountToDegrees(double hoodCount) {
        return OscarMath.map(hoodCount, ShooterValues.HoodBottom, ShooterValues.HoodTop, ShooterValues.HoodMinAngle, ShooterValues.HoodMaxAngle);
    }

    public void trackTarget() {
        setFlywheelRPM(6500);
        setHoodAngle(vision.getSmartHoodAngle());
    }

    public void updateControlLoops(){
        runFlywheelPid();
        runHoodPid();
    }

    private void setFlywheelVoltage(double volts){
        primaryMotor.set(volts / 12);
    }

    public void setFlyheelNeutralMode(NeutralMode mode) {
        primaryMotor.setNeutralMode(mode);
        secondaryMotor.setNeutralMode(mode);
    }

    public void setFeederNeutralMode(NeutralMode mode) {
        feederMotor.setNeutralMode(mode);
    }

    public double getFlywheelRPM_Encoder() {
        return (flywheelEncoder.getRate() / 2048) * 60;
    }

}
