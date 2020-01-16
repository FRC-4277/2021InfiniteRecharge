/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Add your docs here.
 */
public class XboxTrigger extends Trigger {
    private static final double DEFAULT_THRESHOLD = 0.05;
    private XboxController controller;
    private Hand hand;
    private double threshold;

    public XboxTrigger(XboxController controller, Hand hand) {
        this(controller, hand, DEFAULT_THRESHOLD);
    }

    public XboxTrigger(XboxController controller, Hand hand, double threshold) {
        this.controller = controller;
        this.hand = hand;
        this.threshold = threshold;
    }

    @Override
    public boolean get() {
        return controller.getTriggerAxis(Hand.kLeft) >= threshold;
    }
}
