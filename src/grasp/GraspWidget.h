/**
 * @file GraspWidget.h
 */

#pragma once

#include <QtGui>
#include <QtGui/QWidget>
#include "ui_graspInterface.h"

#include "GraspCmd.h"

#include <ros/ros.h>

class GraspWidget : public QWidget, private Ui::grasp {

    Q_OBJECT
	public:
    GraspWidget( QWidget *_parent = 0 );
    ~GraspWidget();
    
    public Q_SLOTS:

    void on_pushButton_close_clicked( bool check );
    void on_pushButton_open_clicked( bool check );

 public:
    GraspCmd graspCommand;
    
};
