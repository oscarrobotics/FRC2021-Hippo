package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.PIDController;
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
    private PIDController flywheelPID = new PIDController(ShooterValues.FlywheelkP, 0, 0);
    private PIDController feederPID = new PIDController(ShooterValues.FeedkP, 0, 0);

    // hood
    private double hoodTargetDegrees, hoodActualDegrees, hoodCount, hoodPIDEffort, hoodFFEffort;
    private final NetworkTableEntry dash_hoodTargetDegrees, dash_hoodActualDegrees, dash_hoodCount, dash_hoodPIDEffort, dash_hoodFFEffort;

    // flywheel
    private double flywheelTargetRPM, flywheelActualRPM, flywheelMotorRPM, flywheelPIDEffort, flywheelFFEffort;
    private final NetworkTableEntry dash_wheelTargetRPM, dash_wheelActualRPM, dash_wheelMotorRPM, dash_wheelFFEffort, dash_wheelPIDEffort;

    // feeder
    private double feedTargetRPM, feedActualRPM, feedPIDEffort, feedFFEffort;
    private final NetworkTableEntry dash_feedTargetRPM, dash_feedActualRPM, dash_feedPIDEffort, dash_feedFFEffort;

    private final CANSparkMax primaryMotor, secondaryMotor, feederMotor, hoodMotor;

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

        primaryMotor.setInverted(true);
        secondaryMotor.follow(primaryMotor, true);

        primaryMotor.limitInputCurrent(55);
        secondaryMotor.limitInputCurrent(55);
        feederMotor.limitInputCurrent(25);
        hoodMotor.limitInputCurrent(15); //CHANGE NUMBER

        zeroHood();
        hoodTargetDegrees = ShooterValues.HoodMinAngle;

        // dashboard

        // feeder
        dash_feedTargetRPM = DashboardManager.addTabItem(this, "Feeder/Target RPM", 0.0);
        dash_feedActualRPM = DashboardManager.addTabItem(this, "Feeder/Actual RPM", 0.0);
        dash_feedFFEffort = DashboardManager.addTabItem(this, "Feeder/FF", 0.0);
        dash_feedPIDEffort = DashboardManager.addTabItem(this, "Feeder/PID", 0.0);

        // flywheel
        dash_wheelTargetRPM = DashboardManager.addTabItem(this, "Flywheel/Target RPM", 0.0);
        dash_wheelActualRPM = DashboardManager.addTabItem(this, "Flywheel/Actual RPM", 0.0);
        dash_wheelMotorRPM = DashboardManager.addTabItem(this, "Flywheel/Motor RPM", 0.0);
        dash_wheelFFEffort = DashboardManager.addTabItem(this, "Flywheel/FF", 0.0);
        dash_wheelPIDEffort = DashboardManager.addTabItem(this, "Flywheel/PID", 0.0);

        dash_hoodTargetDegrees = DashboardManager.addTabItem(this, "Hood/Target Deg", 0.0);
        dash_hoodActualDegrees = DashboardManager.addTabItem(this, "Hood/Actual Deg", 0.0);
        dash_hoodCount = DashboardManager.addTabItem(this, "Hood/EncCount", 0.0);
        dash_hoodPIDEffort = DashboardManager.addTabItem(this, "Hood/PID", 0.0);
        dash_hoodFFEffort = DashboardManager.addTabItem(this, "Hood/FF", 0.0);

        initSuccessful = primaryMotor.getCANConnection() && secondaryMotor.getCANConnection() && feederMotor.getCANConnection();
    }

    private void updateDashboardData() {
        dash_wheelTargetRPM.setDouble(flywheelTargetRPM);
        dash_wheelActualRPM.setDouble(getFlywheelRPM_Encoder());
        dash_wheelMotorRPM.setDouble(primaryMotor.getSensorVelocity());
        dash_wheelPIDEffort.setDouble(flywheelPIDEffort);
        dash_wheelFFEffort.setDouble(flywheelFFEffort);

        dash_hoodTargetDegrees.setDouble(hoodTargetDegrees);
        dash_hoodActualDegrees.setDouble(hoodActualDegrees);
        dash_hoodCount.setDouble(hoodMotor.getSensorPosition());
        dash_hoodPIDEffort.setDouble(hoodPIDEffort);
        dash_hoodFFEffort.setDouble(hoodFFEffort);

        dash_feedActualRPM.setDouble(feedActualRPM);
        dash_feedTargetRPM.setDouble(feedTargetRPM);
        dash_feedFFEffort.setDouble(feedFFEffort);
        dash_feedPIDEffort.setDouble(feedPIDEffort);
    }

    @Override
    public void periodic(){
        updateDashboardData();
    }

    public void zeroHood() {
        hoodMotor.rezeroSensor();
    }

    private void runFlywheelPid() {
        flywheelActualRPM = getFlywheelRPM_Encoder();

        if (flywheelTargetRPM != 0) {
            flywheelFFEffort = ShooterValues.FlyWheelFF.calculate(flywheelTargetRPM) / 12.0;
            flywheelPIDEffort =  flywheelPID.calculate(flywheelActualRPM, flywheelTargetRPM) / 12.0;
        } else {
            flywheelFFEffort = 0;
            flywheelPIDEffort = 0;
        }

        primaryMotor.set(flywheelFFEffort + flywheelPIDEffort);
    }

    private void runFeederPid() {
        feedActualRPM = feederMotor.getSensorVelocity();

        if (feedTargetRPM != 0) {
            feedFFEffort = ShooterValues.FeederFF.calculate(feedTargetRPM);
            feedPIDEffort = feederPID.calculate(feedActualRPM, feedTargetRPM);
        } else {
            feedFFEffort = 0;
            feedPIDEffort = 0;
        }

        feederMotor.set(feedPIDEffort + feedFFEffort);
    }

    private void runHoodPid() {
        hoodActualDegrees = getHoodDegrees();
        hoodPIDEffort = hoodPID.calculate(hoodActualDegrees, hoodTargetDegrees);
        hoodMotor.set(hoodPIDEffort / RobotController.getBatteryVoltage());
    }

    void setFeedRPM(double rpm) {
        feedTargetRPM = rpm;
    }

    void setFlywheelRPM(double wheelTargetRPM) {
        flywheelTargetRPM = wheelTargetRPM;
    }

    public void setHoodAngle(double degrees) {
        if (degrees > 61) degrees = 61;
        hoodTargetDegrees = degrees;
    }

    private double getHoodDegrees() {
        hoodCount = hoodMotor.getSensorPosition();
        return hoodCountToDegrees(hoodCount);
    }

    private double hoodCountToDegrees(double hoodCount) {
        return OscarMath.map(hoodCount, ShooterValues.HoodBottom, ShooterValues.HoodTop, ShooterValues.HoodMinAngle, ShooterValues.HoodMaxAngle);
    }

    void trackTarget() {
        setFlywheelRPM(6500);
//        setHoodAngle(vision.getSmartHoodAngle());
//        setHoodAngle(45);
    }

    public void updateControlLoops(){
        runFlywheelPid();
        runFeederPid();
        runHoodPid();
    }

    public void setFlyheelNeutralMode(NeutralMode mode) {
        primaryMotor.setNeutralMode(mode);
        secondaryMotor.setNeutralMode(mode);
    }

    public void setFeederNeutralMode(NeutralMode mode) {
        feederMotor.setNeutralMode(mode);
    }

    private double getFlywheelRPM_Encoder() {
        return (flywheelEncoder.getRate() / 2048) * 60;
    }

    public void setHoodNeutralMode(NeutralMode mode) {
        hoodMotor.setNeutralMode(mode);
    }
}
