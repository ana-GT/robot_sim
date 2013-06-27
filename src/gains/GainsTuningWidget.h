/**
 * @file GainsTuningWidget.h
 */

#pragma once

#include <QtGui>
#include <QtGui/QWidget>
#include "ui_gainsTuningInterface.h"

#include "GainsTuningCmd.h"

#include <ros/ros.h>

class GainsTuningWidget : public QWidget, private Ui::gainsTuning {

    Q_OBJECT
	public:
    GainsTuningWidget( QWidget *_parent = 0 );
    ~GainsTuningWidget();
    
    public Q_SLOTS:

    void on_pushButton_setGains_Legs_clicked( bool check );
    void on_pushButton_getGains_Legs_clicked( bool check );
    
    void on_pushButton_setGains_Arms_clicked( bool check );
    void on_pushButton_getGains_Arms_clicked( bool check );

    void on_pushButton_setGains_TorsoNeck_clicked( bool check );
    void on_pushButton_getGains_TorsoNeck_clicked( bool check );

 public:
    GainsTuningCmd gtc;
};
