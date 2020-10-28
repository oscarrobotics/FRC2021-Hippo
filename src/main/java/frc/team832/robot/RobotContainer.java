package frc.team832.robot;

import frc.team832.robot.subsystems.TemplateSubsystem;

public class RobotContainer {

    public final TemplateSubsystem templateSubsystem;

    public RobotContainer() {
        templateSubsystem = new TemplateSubsystem();

        if (templateSubsystem.initializedSuccessfully) {
            System.out.println("Template Subsys - INIT OK");
        } else {
            System.out.println("Template Subsys - INIT FAIL");
        }
    }
}