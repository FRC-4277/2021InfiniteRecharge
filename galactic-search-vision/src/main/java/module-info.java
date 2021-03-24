module GalacticSearchVision {
  requires javafx.controls;
  requires javafx.web;
  requires javafx.fxml;
  requires javafx.graphics;
  requires javafx.swing;
  requires ntcore.java;
  requires cscore.java;
  requires cameraserver.java;
  requires opencv;
  requires java.desktop;

  exports frc4277.galacticvision;
  exports frc4277.galacticvision.controllers;
  exports frc4277.galacticvision.pipelines;
  exports frc4277.galacticvision.util;
}
