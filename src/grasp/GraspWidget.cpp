/**
 * @function GraspWidget.cpp
 */
#include "GraspWidget.h"

/**
 * @function GraspWidget
 * @brief Constructor
 */
GraspWidget::GraspWidget( QWidget* _parent ) 
    :QWidget( _parent ) {
    
    // Setup our ui
    setupUi(this);

    // Setup commander
    graspCommand.init( std::string("http://localhost:11311"),
		       std::string("143.215.204.77"),
		       std::string("graspCommand") );
}

/**
 * @function ~GraspWidget
 * @brief Destructor
 */
GraspWidget::~GraspWidget() {

}

/**
 * @function 
 * @brief
 */
void GraspWidget::on_pushButton_close_clicked( bool check ) {
    
    graspCommand.sendBasicCommand( GraspCmd::CLOSED_BOTH );
    printf("Closed hand \n");
}

/**
 * @function 
 * @brief
 */
void GraspWidget::on_pushButton_open_clicked( bool check ) {

 graspCommand.sendBasicCommand( GraspCmd::OPEN_BOTH );
 printf("Opened hand \n");
}
