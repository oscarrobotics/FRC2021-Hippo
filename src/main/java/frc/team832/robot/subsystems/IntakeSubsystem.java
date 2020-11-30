package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.robot.Constants.IntakeValues;
import frc.team832.robot.Constants.PneumaticsValues;

public class IntakeSubsystem extends SubsystemBase {

    public final boolean initSuccessful;

    private final CANTalonFX intakeMotor = new CANTalonFX(IntakeValues.INTAKE_MOTOR_CAN_ID);
    private final Solenoid intakePistons = new Solenoid(PneumaticsValues.PCM_MODULE_NUM, PneumaticsValues.INTAKE_SOLENOID_ID);

    public IntakeSubsystem(GrouchPDP pdp) {
        setName("Intake");
        DashboardManager.addTab(this);

        initSuccessful = intakeMotor.getCANConnection();
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public void setPosition(boolean extended) {
        intakePistons.set(extended);
    }
}
