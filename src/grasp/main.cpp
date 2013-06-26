#include <QtGui>
#include "GraspWidget.h"


int main( int argc, char* argv[] ) { 

    QApplication app( argc, argv );
    
    GraspWidget gw;
    gw.show();
    app.connect( &app, SIGNAL(lastWindowClosed()),
		 &app, SLOT(quit()) );

    int result = app.exec();

    return result;
}
